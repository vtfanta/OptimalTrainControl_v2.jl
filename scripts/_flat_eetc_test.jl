using OptimalTrainControl

track = Track(
    length = 10e3
)

train = Train(
    v -> 1/v,
    v -> -1/v,
    (1e-2, 0., 1.5e-5),
    0.6
)

T = 100. # seconds
prob = EETCProblem(T, train, track, MaxP, 1.0, Float64[])
OptimalTrainControl._solve(prob, 10.)