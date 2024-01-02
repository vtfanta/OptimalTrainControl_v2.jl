using OptimalTrainControl_v2
using Test

@testset "Track utilities" begin
    t = ph.Track(
        1e3,
        100.0,
        [0.0, 200.0, 500.0],
        [0.0, 1.0, 2.0],
        [0.0, 30.0, 600.0],
        [80., 85., 95.]
    )

    @test gradient(t, 94) ≈ 0
    @test gradient(t, 0) ≈ 0
    @test gradient(t, t.length) ≈ 2
    @test gradient(t, 250) ≈ 1
    @test gradient(t, 700) ≈ 2
    @test speedlimit(t, 20) ≈ 80
    @test speedlimit(t, 50) ≈ 85
    @test speedlimit(t, 700) ≈ 95
    @test speedlimit(t, 0) ≈ 80
    @test speedlimit(t, t.length) ≈ 95

    # check if track bounds checked correctly
    @test isvalidposition(t, -0.1) == false
    @test isvalidposition(t, t.length+1) == false
    @test isvalidposition(t, 10)

    @test altitude(t, 50) ≈ t.altitude
    @test altitude(t, 250) ≈ t.altitude + 50
    @test altitude(t, 600) ≈ t.altitude + 300 + 200
    @test altitude(t, t.length) ≈ t.altitude + 300 + 2*500

    @test all(ph.segmentize!(t) .≈ [0,30,200,500,600])

    @test g(t, 250) ≈ -9.81*sqrt(2)/2
end

@testset "Train utilities" begin
    train = Train(
        v -> 1/v,
        v -> -1/v,
        (0.00675, 0.0, 0.00005),
        0.2
    )

    @test r(train, 15) ≈ 0.018
end
