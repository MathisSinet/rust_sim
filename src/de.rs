use core::f64;

use crate::s;
use crate::utils::*;

#[derive(Debug)]
struct MaxXValue {
    power: f64,
    basevalue: f64,
}

impl Value for MaxXValue {
    fn recompute(&self, level: u32) -> f64 {
        self.basevalue.log10() + level as f64 * self.power.log10()
    }
}

#[derive(Debug)]
struct MValue {}
impl Value for MValue {
    fn recompute(&self, level: u32) -> f64 {
        2f64.log10() * (level as i32 - 256) as f64
    }
}

#[derive(Debug)]
struct A2Value {
    stepwise_part: StepwiseValue,
}

impl Value for A2Value {
    fn recompute(&self, level: u32) -> f64 {
        self.stepwise_part.recompute(level) - 10f64.log10()
    }
}

#[derive(Debug)]
struct DEvars {
    n: Variable<ExponentialCost, ExponentialValue>,
    m: Variable<ExponentialCost, MValue>,
    a0: Variable<FirstFreeCost<CompositeCost<ExponentialCost, ExponentialCost>>, StepwiseValue>,
    a1: Variable<ExponentialCost, StepwiseValue>,
    a2: Variable<ExponentialCost, A2Value>,
    max_x: Variable<CompositeCost<ExponentialCost, ExponentialCost>, MaxXValue>,
}

impl DEvars {
    fn init() -> Self {
        DEvars {
            n: Variable::new(
                ExponentialCost::new(200., 2.2),
                ExponentialValue::new(2f64.powf(0.3)),
            ),
            m: Variable::new(ExponentialCost::new_fullbase(200., 1000.), MValue {}),
            a0: Variable::new(
                FirstFreeCost {
                    model: CompositeCost {
                        model1: ExponentialCost::new(3., 1.4),
                        model2: ExponentialCost::new_fullbase(640., 5.),
                        cutoff: 4377,
                    },
                },
                StepwiseValue::new(2.2, 5),
            ),
            a1: Variable::new(ExponentialCost::new(50., 1.74), StepwiseValue::new(3., 7)),
            a2: Variable::new(
                ExponentialCost::new(1e85, 20.),
                A2Value {
                    stepwise_part: StepwiseValue::new(1.5, 11),
                },
            ),
            max_x: Variable::new(
                CompositeCost {
                    model1: ExponentialCost::new(1e7, 2f64.powf(12.)),
                    model2: ExponentialCost::new(1e101, 2f64.powf(19.5)),
                    cutoff: 26,
                },
                MaxXValue {
                    power: 24.5,
                    basevalue: 1024.,
                },
            ),
        }
    }

    fn getm(&mut self, id: usize) -> &mut dyn VariableTrait {
        match id {
            0 => &mut self.n,
            1 => &mut self.m,
            2 => &mut self.a0,
            3 => &mut self.a1,
            4 => &mut self.a2,
            5 | _ => &mut self.max_x,
        }
    }

    fn get(&self, id: usize) -> &dyn VariableTrait {
        match id {
            0 => &self.n,
            1 => &self.m,
            2 => &self.a0,
            3 => &self.a1,
            4 => &self.a2,
            5 | _ => &self.max_x,
        }
    }

    fn set(&mut self, lvls: [u32; 6]) {
        for i in 0..6 {
            self.getm(i).set(lvls[i]);
        }
    }
}

#[derive(Clone, Debug)]
pub struct DEdata {
    pub do_coasting: bool,
}

impl Copy for DEdata {}

pub struct DEstate {
    pub levels: [u32; 6],
    pub tvar: f64,
    pub x: f64,
    pub q: f64,
}

#[derive(Debug)]
pub struct DE {
    data: TheoryData,
    pub dedata: DEdata,
    pub goal: f64,
    rho: f64,
    maxrho: f64,
    multiplier: f64,
    tvar: f64,
    x: f64,
    q: f64,

    vars: DEvars,
    varbuys: Vec<VarBuy>,
    t: f64,
    dt: f64,
    ddt: f64,
}

impl DE {
    pub fn new(data: TheoryData, goal: f64, state: Option<DEstate>) -> Self {
        let mut de = DE {
            data: data,
            dedata: DEdata { do_coasting: true },
            goal: goal,

            rho: 0.,
            maxrho: 0.,
            multiplier: 0.,
            tvar: 0.,
            x: 0.,
            q: 0.,
            vars: DEvars::init(),
            varbuys: Vec::new(),

            t: 0.,
            dt: 1.5,
            ddt: 1.00001,
        };

        match state {
            None => (),
            Some(state) => {
                de.vars.set(state.levels);
                de.tvar = state.tvar;
                de.x = state.x;
                de.q = state.q;
            }
        };

        de.rho = de.data.rho;
        de.multiplier = de.get_multiplier(de.data.tau);

        de
    }

    pub fn fork(&self) -> Self {
        let mut new: DE = DE {
            data: self.data,
            dedata: self.dedata,
            goal: self.goal,
            rho: self.rho,
            maxrho: self.maxrho,
            multiplier: self.multiplier,
            tvar: self.tvar,
            x: self.x,
            q: self.q,
            vars: DEvars::init(),
            varbuys: self.varbuys.clone(),
            t: self.t,
            dt: self.dt,
            ddt: self.ddt,
        };

        for i in 0..6 {
            new.vars.getm(i).set(self.vars.get(i).get_level())
        }

        new
    }

    fn get_multiplier(&self, tau: f64) -> f64 {
        tau * 0.4 - 4f64.log10()
    }

    fn eval_ratio(&self, id: usize) -> BuyEval {
        let mut next_coast = self.vars.max_x.cost;
        if self.dedata.do_coasting {
            next_coast = next_coast.min(self.goal)
        };

        if match id {
            0 => self.vars.n.cost + 5f64.log10() < next_coast,
            1 => self.vars.m.cost + 10f64.log10() < next_coast && self.maxrho * 0.4 < self.data.tau,
            2 => self.t < 60.,
            3 => self.vars.a1.cost + ((5 + self.vars.a1.level % 7) as f64).log10() < next_coast,
            4 => false,
            5 => true,
            _ => false,
        } {
            BuyEval::BUY
        } else {
            BuyEval::SKIP
        }
    }

    fn tick(&mut self) {
        let logdt = self.dt.log10();
        let vn = self.vars.n.value * (1.2 - 0.6 * 0.);
        let va0 = self.vars.a0.value * 3.;

        self.tvar = log10add(self.tvar, self.multiplier + logdt);
        self.x = log10add(
            self.x,
            self.vars.n.value
                + (log10add(f64::consts::E.log10(), va0 - vn) * 10f64.ln()).log10()
                + logdt,
        );
        self.x = self.x.min(self.vars.max_x.value);
        self.q = log10add(
            self.q,
            self.vars.a1.value + self.x + self.vars.m.value - self.tvar + logdt,
        );

        let rhodot = log10add(
            2f64.log10() + self.vars.a1.value + self.x,
            self.vars.a0.value,
        ) + self.vars.n.value
            + 0.1 * self.q;
        self.rho = log10add(self.rho, rhodot + self.multiplier + logdt);

        self.maxrho = self.maxrho.max(self.rho);

        self.t += self.dt / 1.5;
        self.dt *= self.ddt;
    }

    pub fn simulate(&mut self) -> SimRes {
        while self.maxrho < self.goal {
            self.tick();
            //println!("{:?}", self);
            //println!("{} {} {} {}", self.rho, self.maxrho, self.x, self.q);
            self.buy();
        }
        //
        //println!("{}", get_time_string(self.t));

        SimRes {
            t: self.t,
            var_buys: Some(self.varbuys.clone()),
        }
    }

    fn buy(&mut self) {
        let mut cost: f64;
        let mut ratio_eval: BuyEval;
        let names = ["n", "m", "a0", "a1", "a2", "max x"];

        for i in (0..6).rev() {
            cost = self.vars.get(i).get_cost();
            while self.rho > cost {
                ratio_eval = self.eval_ratio(i);
                if ratio_eval == BuyEval::SKIP {
                    break;
                }

                self.rho = log10sub(self.rho, cost);
                self.vars.getm(i).buy();
                cost = self.vars.get(i).get_cost();

                if self.maxrho > self.data.tau * 2.5 - 5. {
                    self.varbuys.push(VarBuy {
                        symb: s!(names[i]),
                        lvl: self.vars.get(i).get_level(),
                        t: self.t,
                    })
                }
            }
        }
    }
}
