use crate::s;
use crate::utils::*;

struct T2vars {
    dq1: Variable<FirstFreeCost<ExponentialCost>, StepwiseValue>,
    dq2: Variable<ExponentialCost, StepwiseValue>,
    dq3: Variable<ExponentialCost, StepwiseValue>,
    dq4: Variable<ExponentialCost, StepwiseValue>,
    dr1: Variable<ExponentialCost, StepwiseValue>,
    dr2: Variable<ExponentialCost, StepwiseValue>,
    dr3: Variable<ExponentialCost, StepwiseValue>,
    dr4: Variable<ExponentialCost, StepwiseValue>,
}

impl T2vars {
    fn init() -> Self {
        T2vars {
            dq1: Variable::new(
                FirstFreeCost {
                    model: ExponentialCost::new(10., 2.),
                },
                StepwiseValue::new(2., 10),
            ),
            dq2: Variable::new(ExponentialCost::new(5e3, 2.), StepwiseValue::new(2., 10)),
            dq3: Variable::new(ExponentialCost::new(3e25, 3.), StepwiseValue::new(2., 10)),
            dq4: Variable::new(ExponentialCost::new(8e50, 4.), StepwiseValue::new(2., 10)),
            dr1: Variable::new(ExponentialCost::new(2e6, 2.), StepwiseValue::new(2., 10)),
            dr2: Variable::new(ExponentialCost::new(3e9, 2.), StepwiseValue::new(2., 10)),
            dr3: Variable::new(ExponentialCost::new(4e25, 3.), StepwiseValue::new(2., 10)),
            dr4: Variable::new(ExponentialCost::new(5e50, 4.), StepwiseValue::new(2., 10)),
        }
    }

    fn getm(&mut self, id: usize) -> &mut dyn VariableTrait {
        match id {
            0 => &mut self.dq1,
            1 => &mut self.dq2,
            2 => &mut self.dq3,
            3 => &mut self.dq4,
            4 => &mut self.dr1,
            5 => &mut self.dr2,
            6 => &mut self.dr3,
            _ => &mut self.dr4,
        }
    }

    fn get(&self, id: usize) -> &dyn VariableTrait {
        match id {
            0 => &self.dq1,
            1 => &self.dq2,
            2 => &self.dq3,
            3 => &self.dq4,
            4 => &self.dr1,
            5 => &self.dr2,
            6 => &self.dr3,
            _ => &self.dr4,
        }
    }

    fn set(&mut self, lvls: [u32; 8]) {
        for (i, level) in lvls.iter().enumerate() {
            self.getm(i).set(*level);
        }
    }
}

#[derive(Clone)]
struct T2data {
    caps: [u32; 8],
}

impl Copy for T2data {}

pub struct T2state {
    pub levels: [u32; 8],
    pub layers: [f64; 8],
}

pub struct T2 {
    data: TheoryData,
    t2data: T2data,
    goal: f64,
    rho: f64,
    maxrho: f64,
    multiplier: f64,
    layers: [f64; 8],
    vars: T2vars,
    varbuys: Vec<VarBuy>,

    t: f64,
    dt: f64,
    ddt: f64,
    depth: u32,

    best_res: SimRes,
}

impl T2 {
    pub fn new(data: TheoryData, goal: f64, state: Option<T2state>) -> Self {
        let mut t2: T2 = T2 {
            data: data,
            t2data: T2data {
                caps: [u32::MAX; 8],
            },
            goal: goal,
            rho: 0.,
            maxrho: 0.,
            multiplier: 0.,
            layers: [0.; 8],
            vars: T2vars::init(),
            varbuys: Vec::new(),

            t: 0.,
            dt: 1.5,
            ddt: 1.0001,
            depth: 0,

            best_res: SimRes::default(),
        };

        match state {
            None => (),
            Some(state) => {
                t2.vars.set(state.levels);
                t2.layers = state.layers
            }
        };

        t2.rho = t2.data.rho;
        t2.multiplier = t2.get_multiplier(t2.data.tau, t2.data.students);

        t2
    }

    fn fork(&self) -> Self {
        let mut new: T2 = T2 {
            data: self.data,
            t2data: self.t2data,
            goal: self.goal,
            rho: self.rho,
            maxrho: self.maxrho,
            multiplier: self.multiplier,
            layers: self.layers,
            vars: T2vars::init(),
            varbuys: self.varbuys.clone(),
            t: self.t,
            dt: self.dt,
            ddt: self.ddt,
            depth: self.depth + 1,

            best_res: SimRes::default(),
        };

        for i in 0..8 {
            new.vars.getm(i).set(self.vars.get(i).get_level())
        }

        new
    }

    fn get_multiplier(&self, tau: f64, students: u32) -> f64 {
        3. * (students as f64 / 20.).log10() + 0.198 * tau - 2.
    }

    fn eval_coast_one(&self, dist: f64, lbound: f64, ubound: f64) -> BuyEval {
        if dist > ubound {
            BuyEval::BUY
        } else if dist > lbound {
            BuyEval::FORK
        } else {
            BuyEval::SKIP
        }
    }

    fn eval_coast(&self, id: usize, cost: f64) -> BuyEval {
        let dist: f64 = self.goal - cost;
        if dist > 7. {
            return BuyEval::BUY;
        }
        match id {
            0 | 4 => self.eval_coast_one(dist, 1.1, 2.),
            1 | 5 => self.eval_coast_one(dist, 1.8, 3.),
            2 | 6 => self.eval_coast_one(dist, 2.9, 4.1),
            3 | 7 => self.eval_coast_one(dist, 4.9, 6.1),
            _ => BuyEval::SKIP,
        }
    }

    fn eval_ratio(&self, id: usize) -> BuyEval {
        return BuyEval::BUY;

        if self.goal - self.maxrho < 8. {
            return BuyEval::BUY;
        }

        let mut costs: [f64; 8] = [0.; 8];
        let mut levels: [u32; 8] = [1; 8];
        for i in 0..8 {
            costs[i] = self.vars.get(i).get_cost();
            levels[i] = self.vars.get(i).get_level();
            match i {
                0 | 4 => costs[i] += (1f64 + 0.24 * ((levels[i] % 10) as f64)).log10(),
                1 | 5 => costs[i] += (1f64 + 0.18 * ((levels[i] % 10) as f64)).log10(),
                2 | 6 => costs[i] += (1f64 + 0.12 * ((levels[i] % 10) as f64)).log10(),
                _ => costs[i] += (1f64 + 0.05 * ((levels[i] % 10) as f64)).log10(),
            }
        }

        let mut bestind = 0;
        for i in 1..8 {
            if costs[i] < costs[bestind] {
                bestind = i;
            }
        }

        if bestind == id {
            BuyEval::BUY
        } else {
            BuyEval::SKIP
        }
    }

    fn tick(&mut self) {
        let logdt: f64 = self.dt.log10();

        self.layers[0] = log10add(self.layers[0], self.vars.dq1.value + self.layers[1] + logdt);
        self.layers[1] = log10add(self.layers[1], self.vars.dq2.value + self.layers[2] + logdt);
        self.layers[2] = log10add(self.layers[2], self.vars.dq3.value + self.layers[3] + logdt);
        self.layers[3] = log10add(self.layers[3], self.vars.dq4.value + logdt);
        self.layers[4] = log10add(self.layers[4], self.vars.dr1.value + self.layers[5] + logdt);
        self.layers[5] = log10add(self.layers[5], self.vars.dr2.value + self.layers[6] + logdt);
        self.layers[6] = log10add(self.layers[6], self.vars.dr3.value + self.layers[7] + logdt);
        self.layers[7] = log10add(self.layers[7], self.vars.dr4.value + logdt);

        self.rho = log10add(
            self.rho,
            self.layers[0] * 1.15 + self.layers[4] * 1.15 + self.multiplier + logdt,
        );
        self.maxrho = self.maxrho.max(self.rho);

        self.t += self.dt / 1.5;
        self.dt *= self.ddt;
    }

    pub fn simulate(&mut self) -> SimRes {
        while self.maxrho < self.goal {
            self.tick();
            self.buy();
        }

        if self.t < self.best_res.t {
            SimRes {
                t: self.t,
                var_buys: Some(self.varbuys.clone()),
            }
        } else {
            SimRes {
                t: self.best_res.t,
                var_buys: self.best_res.var_buys.clone(),
            }
        }
    }

    fn buy(&mut self) {
        let mut cost: f64;
        let mut coast_eval: BuyEval;
        let mut ratio_eval: BuyEval;
        let names = ["r4", "r3", "r2", "r1", "q4", "q3", "q2", "q1"];
        let ids: [usize; 8] = [7, 6, 5, 4, 3, 2, 1, 0];

        for i in 0..8 {
            //if self.t2data.skip[ids[i]] { continue; }
            if self.vars.get(ids[i]).get_level() >= self.t2data.caps[ids[i]] {
                continue;
            }
            cost = self.vars.get(ids[i]).get_cost();
            while self.rho > cost {
                coast_eval = self.eval_coast(ids[i], cost);
                ratio_eval = self.eval_ratio(ids[i]);
                if coast_eval != BuyEval::SKIP {
                    if ratio_eval == BuyEval::SKIP {
                        break;
                    }
                    if coast_eval == BuyEval::FORK {
                        let mut fork: T2 = self.fork();
                        let lvl: u32 = self.vars.get(ids[i]).get_level();
                        fork.t2data.caps[ids[i]] = lvl;
                        if self.depth <= 2 {
                            println!(
                                "Depth {}; Creating coasting fork for {} lvl {}",
                                self.depth, names[i], lvl
                            );
                        }
                        let res: SimRes = fork.simulate();
                        if res.t < self.best_res.t {
                            self.best_res.t = res.t;
                            self.best_res.var_buys = res.var_buys;
                        }
                    }
                    /*if ratio_eval == BuyEval::FORK {
                        let mut fork: T2 = self.fork();
                        fork.t2data.skip[ids[i]] = true;
                        if self.depth <= 2 {
                            println!("Depth {}; Creating ratio fork for {} lvl {}", self.depth, names[i], self.vars.get(ids[i]).get_level());
                        }
                        let res: SimRes = fork.simulate();
                        if res.t < self.best_res.t {
                            self.best_res.t = res.t;
                            self.best_res.var_buys = res.var_buys;
                        }
                    }*/

                    self.rho = log10sub(self.rho, cost);
                    self.vars.getm(ids[i]).buy();
                    cost = self.vars.get(ids[i]).get_cost();
                    //for j in 0..7 {self.t2data.skip[j] = false;}

                    if self.maxrho > self.goal - 9. {
                        self.varbuys.push(VarBuy {
                            symb: s!(names[i]),
                            lvl: self.vars.get(ids[i]).get_level(),
                            t: self.t,
                        })
                    }
                } else {
                    self.t2data.caps[ids[i]] = self.vars.get(ids[i]).get_level();
                    break;
                }
            }
        }
    }
}
