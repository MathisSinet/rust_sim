use crate::s;
use crate::utils::*;

struct T1vars {
    q1: Variable<FirstFreeCost<ExponentialCost>, StepwiseValue>,
    q2: Variable<ExponentialCost, ExponentialValue>,
    c3: Variable<ExponentialCost, ExponentialValue>,
    c4: Variable<ExponentialCost, ExponentialValue>,
}

impl T1vars {
    fn init() -> Self {
        T1vars {
            q1: Variable::new(
                FirstFreeCost {
                    model: ExponentialCost::new(5., 2.),
                },
                StepwiseValue::new(2., 10),
            ),
            q2: Variable::new(ExponentialCost::new(100., 10.), ExponentialValue::new(2.)),
            c3: Variable::new(
                ExponentialCost::new(1e4, 10f64.powf(4.5)),
                ExponentialValue::new(10.),
            ),
            c4: Variable::new(
                ExponentialCost::new(1e10, 10f64.powi(8)),
                ExponentialValue::new(10.),
            ),
        }
    }

    fn getm(&mut self, id: usize) -> &mut dyn VariableTrait {
        match id {
            0 => &mut self.q1,
            1 => &mut self.q2,
            2 => &mut self.c3,
            _ => &mut self.c4,
        }
    }

    fn get(&self, id: usize) -> &dyn VariableTrait {
        match id {
            0 => &self.q1,
            1 => &self.q2,
            2 => &self.c3,
            _ => &self.c4,
        }
    }

    fn set(&mut self, lvls: [u32; 4]) {
        for (i, level) in lvls.iter().enumerate() {
            self.getm(i).set(*level);
        }
    }
}

#[derive(Clone)]
pub struct T1data {
    pub do_coasting: bool,
}

impl Copy for T1data {}

pub struct T1state {
    pub levels: [u32; 4],
}

pub struct T1 {
    data: TheoryData,
    pub t1data: T1data,
    pub goal: f64,
    rho: f64,
    maxrho: f64,
    multiplier: f64,
    vars: T1vars,
    varbuys: Vec<VarBuy>,

    t: f64,
    dt: f64,
    ddt: f64,
    depth: u32,

    best_res: SimRes,
}

impl T1 {
    pub fn new(data: TheoryData, goal: f64, state: Option<T1state>) -> Self {
        let mut t1: T1 = T1 {
            data: data,
            t1data: T1data { do_coasting: true },
            goal: goal,
            rho: 0.,
            maxrho: 0.,
            multiplier: 0.,
            vars: T1vars::init(),
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
                t1.vars.set(state.levels);
            }
        }

        t1.rho = t1.data.rho;
        t1.multiplier = t1.get_multiplier(t1.data.tau, t1.data.students);

        t1
    }

    pub fn fork(&self) -> Self {
        let mut new: T1 = T1 {
            data: self.data,
            t1data: self.t1data,
            goal: self.goal,
            rho: self.rho,
            maxrho: self.maxrho,
            multiplier: self.multiplier,
            vars: T1vars::init(),
            varbuys: self.varbuys.clone(),
            t: self.t,
            dt: self.dt,
            ddt: self.ddt,
            depth: self.depth + 1,

            best_res: SimRes::default(),
        };

        for i in 0..4 {
            new.vars.getm(i).set(self.vars.get(i).get_level())
        }

        new
    }

    fn get_multiplier(&self, tau: f64, sigma: u32) -> f64 {
        tau * 0.164 - 3f64.log10() + 3. * (sigma as f64 / 20.).log10()
    }

    fn eval_buy(&self, id: usize) -> BuyEval {
        //return BuyEval::BUY;
        let c3term = self.vars.c3.value + 0.2 * self.rho;
        let c4term = self.vars.c4.value + 0.3 * self.rho;
        let term_sum = log10add(c3term, c4term);
        let c3_ratio = 10f64.powf(c3term - term_sum);
        let c4_ratio = 10f64.powf(c4term - term_sum);

        let multipliers: [f64; 4] = [
            (11. + (self.vars.q1.level % 10) as f64) / (10. + (self.vars.q1.level % 10) as f64),
            2.,
            10. * c3_ratio + c4_ratio,
            c3_ratio + 10. * c4_ratio,
        ];

        let mut weighted_costs: [f64; 4] = [0.; 4];

        for i in 0..4 {
            weighted_costs[i] = self.vars.get(i).get_cost()
                + (1. / 0.7)
                    * ((-1. / (multipliers[i] * (multipliers[i].powf(1. / 0.3) - 1.).powf(0.7))
                        + 1. / (1. - 1. / multipliers[i].powf(1. / 0.3)).powf(0.7))
                        / (1. - 1. / multipliers[i]))
                        .log10()
        }

        if self.t1data.do_coasting && weighted_costs[id] > self.goal {
            return BuyEval::SKIP;
        };

        let min_wcost = weighted_costs[0]
            .min(weighted_costs[1])
            .min(weighted_costs[2])
            .min(weighted_costs[3]);
        let mult2 = multipliers[id].powf(10. / 3.);

        //println!("{} {} {}", id, weighted_costs[id], min_wcost);

        if weighted_costs[id] < min_wcost + 0.0001
            && self.rho > self.vars.get(id).get_cost() + (1. / (1. - 1. / mult2)).log10()
        {
            BuyEval::BUY
        } else {
            BuyEval::SKIP
        }
    }

    fn tick(&mut self) {
        self.rho = log10add(
            self.rho,
            log10add(
                self.vars.c3.value + self.rho * 0.2,
                self.vars.c4.value + self.rho * 0.3,
            ) + self.vars.q1.value
                + self.vars.q2.value
                + self.multiplier
                + self.dt.log10(),
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
        let mut eval: BuyEval;
        let names = ["c4", "c3", "q2", "q1"];
        let ids: [usize; 4] = [3, 2, 1, 0];

        for i in 0..4 {
            cost = self.vars.get(ids[i]).get_cost();
            while self.rho > cost {
                eval = if cost < self.data.tau - 50. {
                    BuyEval::BUY
                } else {
                    self.eval_buy(ids[i])
                };
                if eval == BuyEval::BUY {
                    self.rho = log10sub(self.rho, cost);
                    self.vars.getm(ids[i]).buy();
                    cost = self.vars.get(ids[i]).get_cost();

                    if self.maxrho > self.data.tau - 5. {
                        self.varbuys.push(VarBuy {
                            symb: s!(names[i]),
                            lvl: self.vars.get(ids[i]).get_level(),
                            t: self.t,
                        })
                    }
                } else {
                    break;
                };
            }
        }
    }
}
