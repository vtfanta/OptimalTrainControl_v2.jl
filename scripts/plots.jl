using OptimalTrainControl
# using Plots

train = Train(
        v -> 3/v,
        v -> -3/v,
        (6.75e-3, 0., 5e-5)
    )

track = Track(
        length = 300.,
        altitude = 100.,
        x_gradient = [0.0, 100., 173.2],
        gradient = [2e-3, 0., 1e-3]
    )

prob = TOTCProblem(train, track)

sol = solve(prob)
@show mode2color(sol.phases[end])
plot(sol, label = false, ylabel = "Speed (m/s)", xlabel = "Distance (m)")
# plot!(twinx(), track, ylabel = "Altitude (m)")