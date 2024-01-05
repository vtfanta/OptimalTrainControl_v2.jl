using OptimalTrainControl

track = Track(
    length = 1e3
)

train = Train(
    v -> 1/v,
    v -> -1/v,
    (1e-2, 0., 1.5e-5)
)

T = 100. # seconds

prob = EETCProblem(T, train, track, MaxP)
solve(prob)