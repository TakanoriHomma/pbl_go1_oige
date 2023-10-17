import numpy as np

from optimizers import CEMOptimizer
from PtModel import PtModel

from tqdm import trange

import torch

TORCH_DEVICE = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

def shuffle_rows(arr):
    idxs = np.argsort(np.random.uniform(size=arr.shape), axis=-1)
    return arr[np.arange(arr.shape[0])[:, None], idxs]


class MPC():
    def __init__(self):
        self.dO, self.dU = 22, 12
        self.ac_ub, self.ac_lb = 0.3*np.ones(12),-0.3*np.ones(12)
        self.per = 1
        self.prop_mode = 'TSinf'
        self.npart = 20
        self.plan_hor = 20
        self.iter = 0
        self.optimizer = CEMOptimizer(
            sol_dim=self.plan_hor * self.dU,
            max_iters=5,
            popsize=200,
            num_elites=30,
            lower_bound=np.tile(self.ac_lb, [self.plan_hor]),
            upper_bound=np.tile(self.ac_ub, [self.plan_hor]),
            cost_function=self._compile_cost,
        )

        self.ac_buf = np.array([]).reshape(0, self.dU)
        self.prev_sol = np.tile((self.ac_lb + self.ac_ub) / 2, [self.plan_hor])
        self.init_var = np.tile(np.square(self.ac_ub - self.ac_lb) / 16, [self.plan_hor])
        self.train_in = np.array([]).reshape(0,self.dO+self.dU)
        self.train_targs = np.array([]).reshape(0,self.dO)
        self.model_in=34
        self.model_out=22
        self.num_nets=5
        self.model = PtModel(self.num_nets,self.model_in,self.model_out*2).to(TORCH_DEVICE)
        self.model.optim = torch.optim.Adam(self.model.parameters(), lr=0.001)
        dict_ = torch.load('model/model_weight.pth')
        dict_['inputs_mu'],dict_['inputs_sigma'] = dict_['inputs_mu'].reshape(34),dict_['inputs_sigma'].reshape(34)
        self.model.load_state_dict(dict_)
    
    def obs_preproc(self,obs):
        return obs
    
    def obs_postproc(self,obs, pred):
        assert isinstance(obs, torch.Tensor)
        return pred
    
    def obs_postproc2(self,next_obs):
        assert isinstance(next_obs, torch.Tensor)
        return next_obs
    
    def targ_proc(self,next_obs):
        return next_obs
    
    def obs_cost_fn(self,obs):
        return torch.where(obs[:,4]>0.1,-5*obs[:,4],-3*(obs[:,4]-0.1))\
                + torch.where(torch.abs(obs[:,3])>np.pi/9*2,torch.abs(obs[:,3])-np.pi/9*2,0) \
                + torch.where(torch.abs(obs[:,2])>np.pi/9,2*(torch.abs(obs[:,2])-np.pi/9),0) \
                + torch.where(torch.abs(obs[:,8])>1,2*(torch.abs(obs[:,8])-1),0) \
                + torch.where(torch.abs(obs[:,1])>np.pi/9,2*(torch.abs(obs[:,1])-np.pi/9),0)
    
    def ac_cost_fn(self,acs):
        return 0 * (acs ** 2).sum(dim=1)
                
    def reset(self):
        self.prev_sol = np.tile((self.ac_lb + self.ac_ub) / 2, [self.plan_hor])

    def act(self, obs):
        if self.ac_buf.shape[0] > 0:
            action, self.ac_buf = self.ac_buf[0], self.ac_buf[1:]
            return action
        self.sy_cur_obs = obs
        # popsize_rate = (self.iter+1)/300 + 0.25
        soln = self.optimizer.obtain_solution(self.prev_sol, self.init_var)
        self.prev_sol = np.concatenate([np.copy(soln)[self.per * self.dU:], np.zeros(self.per * self.dU)])
        self.ac_buf = soln[:self.per * self.dU].reshape(-1, self.dU)

        return self.act(obs)

    @torch.no_grad()
    def _compile_cost(self, ac_seqs):

        nopt = ac_seqs.shape[0]

        ac_seqs = torch.from_numpy(ac_seqs).float().to(TORCH_DEVICE)

        # Reshape ac_seqs so that it's amenable to parallel compute
        # Before, ac seqs has dimension (400, 25) which are pop size and sol dim coming from CEM
        ac_seqs = ac_seqs.view(-1, self.plan_hor, self.dU)
        #  After, ac seqs has dimension (400, 25, 1)

        transposed = ac_seqs.transpose(0, 1)
        # Then, (25, 400, 1)

        expanded = transposed[:, :, None]
        # Then, (25, 400, 1, 1)

        tiled = expanded.expand(-1, -1, self.npart, -1)
        # Then, (25, 400, 20, 1)

        ac_seqs = tiled.contiguous().view(self.plan_hor, -1, self.dU)
        # Then, (25, 8000, 1)

        # Expand current observation
        cur_obs = torch.from_numpy(self.sy_cur_obs).float().to(TORCH_DEVICE)
        cur_obs = cur_obs[None]
        cur_obs = cur_obs.expand(nopt * self.npart, -1)

        costs = torch.zeros(nopt, self.npart, device=TORCH_DEVICE)

        # ac_cost = self.ac_cost_fn(ac_seqs).view(-1,self.npart)

        for t in range(self.plan_hor):
            cur_acs = ac_seqs[t]

            next_obs = self._predict_next_obs(cur_obs, cur_acs)

            cost = self.obs_cost_fn(next_obs) + self.ac_cost_fn(cur_acs)

            cost = cost.view(-1, self.npart)

            costs += cost
            cur_obs = self.obs_postproc2(next_obs)

        # costs += ac_cost
        # Replace nan with high cost
        costs[costs != costs] = 1e6

        return costs.mean(dim=1).detach().cpu().numpy()

    def _predict_next_obs(self, obs, acs):
        proc_obs = self.obs_preproc(obs)

        assert self.prop_mode == 'TSinf'

        proc_obs = self._expand_to_ts_format(proc_obs)
        acs = self._expand_to_ts_format(acs)

        inputs = torch.cat((proc_obs, acs), dim=-1)

        mean, var = self.model(inputs)
        #このvarにはaleatoric,epistemic両方の誤差が含まれている 

        predictions = mean + torch.randn_like(mean, device=TORCH_DEVICE) * var.sqrt()

        # TS Optimization: Remove additional dimension
        predictions = self._flatten_to_matrix(predictions)

        return self.obs_postproc(obs, predictions)

    def _expand_to_ts_format(self, mat):
        dim = mat.shape[-1]

        # Before, [10, 5] in case of proc_obs
        reshaped = mat.view(-1, self.model.num_nets, self.npart // self.model.num_nets, dim)
        # After, [2, 5, 1, 5]

        transposed = reshaped.transpose(0, 1)
        # After, [5, 2, 1, 5]

        reshaped = transposed.contiguous().view(self.model.num_nets, -1, dim)
        # After. [5, 2, 5]

        return reshaped

    def _flatten_to_matrix(self, ts_fmt_arr):
        dim = ts_fmt_arr.shape[-1]

        reshaped = ts_fmt_arr.view(self.model.num_nets, -1, self.npart // self.model.num_nets, dim)

        transposed = reshaped.transpose(0, 1)

        reshaped = transposed.contiguous().view(-1, dim)

        return reshaped
