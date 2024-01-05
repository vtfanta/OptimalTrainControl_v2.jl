using OptimalTrainControl
using Plots

train = Train(
        v -> 3/v,
        v -> -3/v,
        (6.75e-3, 0., 5e-5)
    )

track = Track(
    length = 300.,
    altitude = 100.,
    x_gradient = collect(0.:1.:300.),
    gradient = 5e-2*sin.(collect(0.:1.:300.)./20.)
)

timeprob = TOTCProblem(;train, track)

sol = solve(timeprob)

plot(sol, xlabel = "Distance (m)", ylabel = "Speed (m/s)", label = false)