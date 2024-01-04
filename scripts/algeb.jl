using OptimalTrainControl
using Plots

r(v) = 1e-2 + 1.5e-5v^2
ψ(v) = 3e-5v^3
E(V, v) = ψ(V)/v + r(v)

V = 20.