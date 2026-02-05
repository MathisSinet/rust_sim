use crate::s;
use crate::utils::*;

struct T7vars {
    q1: Variable<FirstFreeCost<ExponentialCost>, StepwiseValue>,
    c3: Variable<ExponentialCost, ExponentialValue>,
    c4: Variable<ExponentialCost, ExponentialValue>,
    c5: Variable<ExponentialCost, ExponentialValue>,
    c6: Variable<ExponentialCost, ExponentialValue>,
}

impl T7vars {
    fn init() -> Self {
        T7vars {
            q1: Variable::new(
                FirstFreeCost {
                    model: ExponentialCost::new(500., 1.51572),
                },
                StepwiseValue::new(2., 10),
            ),
            c3: Variable::new(
                ExponentialCost::new(1e5, 63.), 
                ExponentialValue::new(2.)
            ),
            c4: Variable::new(
                ExponentialCost::new(10., 2.82),
                ExponentialValue::new(2.),
            ),
            c5: Variable::new(
                ExponentialCost::new(1e8, 60.),
                ExponentialValue::new(2.),
            ),
            c6: Variable::new(
                ExponentialCost::new(100., 2.81),
                ExponentialValue::new(2.),
            ),
        }
    }

    fn getm(&mut self, id: usize) -> &mut dyn VariableTrait {
        match id {
            0 => &mut self.q1,
            1 => &mut self.c3,
            2 => &mut self.c4,
            3 => &mut self.c5,
            _ => &mut self.c6
        }
    }

    fn get(&self, id: usize) -> &dyn VariableTrait {
        match id {
            0 => &self.q1,
            1 => &self.c3,
            2 => &self.c4,
            3 => &self.c5,
            _ => &self.c6
        }
    }

    fn set(&mut self, lvls: [u32; 5]) {
        for (i, level) in lvls.iter().enumerate() {
            self.getm(i).set(*level);
        }
    }
}

#[derive(Clone)]
pub struct T7data {
    pub do_coasting: bool,
}

impl Copy for T7data {}

pub struct T7state {
    pub levels: [u32; 5],
    pub rho2: f64
}

pub struct T7 {
    data: TheoryData,
    pub t7data: T7data,
    pub goal: f64,
    rho: f64,
    rho2: f64,
    drho13: f64,
    drho23: f64,
    maxrho: f64,
    multiplier: f64,
    vars: T7vars,
    varbuys: Vec<VarBuy>,

    t: f64,
    dt: f64,
    ddt: f64,
    depth: u32,

    best_res: SimRes,
}

impl T7 {
    pub fn new(data: TheoryData, goal: f64, state: Option<T7state>) -> Self {
        let mut t7: T7 = T7 {
            data: data,
            t7data: T7data { do_coasting: true },
            goal: goal,
            rho: 0.,
            rho2: 0.,
            drho13: 0.,
            drho23: 0.,
            maxrho: 0.,
            multiplier: 0.,
            vars: T7vars::init(),
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
                t7.vars.set(state.levels);
                t7.rho2 = state.rho2;
            }
        }

        t7.rho = t7.data.rho;
        t7.multiplier = t7.get_multiplier(t7.data.tau, t7.data.students);

        t7
    }

    pub fn fork(&self) -> Self {
        let mut new: T7 = T7 {
            data: self.data,
            t7data: self.t7data,
            goal: self.goal,
            rho: self.rho,
            rho2: self.rho2,
            drho13: self.drho13,
            drho23: self.drho23,
            maxrho: self.maxrho,
            multiplier: self.multiplier,
            vars: T7vars::init(),
            varbuys: self.varbuys.clone(),
            t: self.t,
            dt: self.dt,
            ddt: self.ddt,
            depth: self.depth + 1,

            best_res: SimRes::default(),
        };

        for i in 0..5 {
            new.vars.getm(i).set(self.vars.get(i).get_level())
        }

        new
    }

    fn get_multiplier(&self, tau: f64, sigma: u32) -> f64 {
        tau * 0.152 + 3. * (sigma as f64 / 20.).log10()
    }

    fn eval_coast(&self, id: usize, cost: f64) -> BuyEval {
        let dist: f64 = self.goal - cost;
        if dist > 1.5 {
            return BuyEval::BUY;
        }
        match id {
            0 | 3 => {
                if dist < 8f64.log10() {
                    BuyEval::SKIP
                } else {
                    BuyEval::BUY
                }
            }
            1 | 2 => {
                if dist < 20f64.log10() {
                    BuyEval::SKIP
                } else {
                    BuyEval::BUY
                }
            }
            4 => {
                if dist < 2f64.log10() {
                    BuyEval::SKIP
                } else {
                    BuyEval::BUY
                }
            }
            _ => BuyEval::SKIP,
        }
    }

    fn eval_ratio(&self, id: usize, cost: f64) -> BuyEval {
        let dist: f64 = self.vars.c6.cost - cost;
        match id {
            0 | 3 => {
                if dist < 4f64.log10() {
                    BuyEval::SKIP
                } else {
                    BuyEval::BUY
                }
            }
            1 | 2 => {
                if dist < 1. {
                    BuyEval::SKIP
                } else {
                    BuyEval::BUY
                }
            }
            4 => BuyEval::BUY,
            _ => BuyEval::SKIP,
        }
    }

    fn tick(&mut self) {
        let drho12 = 1.5f64.log10() + self.vars.c3.value + self.rho / 2.;
        let drho22 = 1.5f64.log10() + self.vars.c5.value + self.rho2 / 2.;
        self.drho13 = (0.5f64.log10() + self.vars.c6.value + self.rho2 / 2. - self.rho / 2.).min(self.drho13 + 2.).min(self.rho + 2.);
        self.drho23 = (0.5f64.log10() + self.vars.c6.value + self.rho / 2. - self.rho2 / 2.).min(self.drho23 + 2.).min(self.rho2 + 2.);
        let dtq1bonus = self.dt.log10() + self.vars.q1.value + self.multiplier;

        self.rho = log10add(self.rho, dtq1bonus + log10add(drho12, self.drho13));
        self.rho2 = log10add(self.rho2, dtq1bonus + log10add(drho22, self.drho23));

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
        let names = ["c6", "c5", "c4", "c3", "q1"];
        let ids: [usize; 5] = [4, 3, 2, 1, 0];

        for i in 0..5 {
            cost = self.vars.get(ids[i]).get_cost();
            while self.rho > cost {
                eval = if cost < self.data.tau - 50. {
                    BuyEval::BUY
                } else {
                    let ev = self.eval_ratio(ids[i], cost);
                    if ev == BuyEval::BUY && self.t7data.do_coasting {
                        self.eval_coast(ids[i], cost)
                    }
                    else {
                        ev
                    }
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