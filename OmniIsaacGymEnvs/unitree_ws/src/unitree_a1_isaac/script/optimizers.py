import numpy as np
import scipy.stats as stats

class CEMOptimizer():

    def __init__(self, sol_dim, max_iters, popsize, num_elites, cost_function,
                 upper_bound=None, lower_bound=None, epsilon=0.0001, alpha=0.25):
        self.sol_dim, self.max_iters, self.popsize, self.num_elites = sol_dim, max_iters, popsize, num_elites

        self.ub, self.lb = upper_bound, lower_bound
        self.epsilon, self.alpha = epsilon, alpha

        self.cost_function = cost_function

        if num_elites > popsize:
            raise ValueError("Number of elites must be at most the population size.")

    def obtain_solution(self, init_mean, init_var,popsize_rate=1):
        mean, var, t = init_mean, init_var, 0
        X = stats.truncnorm(-2, 2, loc=np.zeros_like(mean), scale=np.ones_like(var))

        while (t < self.max_iters) and np.max(var) > self.epsilon:
            lb_dist, ub_dist = mean - self.lb, self.ub - mean
            constrained_var = np.minimum(np.minimum(np.square(lb_dist / 2), np.square(ub_dist / 2)), var)

            samples = X.rvs(size=[int(self.popsize*popsize_rate), self.sol_dim]) * np.sqrt(constrained_var) + mean
            samples = samples.astype(np.float32)
            costs = self.cost_function(samples)
            elites = samples[np.argsort(costs)][:int(self.num_elites*popsize_rate)]

            new_mean = np.mean(elites, axis=0)
            new_var = np.var(elites, axis=0)
            mean = self.alpha * mean + (1 - self.alpha) * new_mean
            var = self.alpha * var + (1 - self.alpha) * new_var

            t += 1
            

        return mean
