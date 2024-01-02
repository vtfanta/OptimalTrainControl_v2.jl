using OptimalTrainControl

train = Train(
        v -> 3/v,
        v -> -3/v,
        (6.75e-3, 0., 5e-5)
    )

track = Track(
    length = 300.,
    altitude = 100.,
    x_gradient = [0.0],
    gradient = [2e-3]
)

timeprob = TOTCProblem(train, track)

sol = solve(timeprob)

plot(sol.odesol.t, sol.odesol[2,:])