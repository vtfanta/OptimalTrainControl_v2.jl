using OptimalTrainControl
using OrdinaryDiffEq
using Plots
using Roots
using StaticArrays

r(v) = 1e-2 + 1.5e-5v^2
ψ(v) = 3e-5v^3
E(V, v) = ψ(V)/v + r(v)

function _odefun(s::A, p::EETCProblem, x::T) where {T<:Real, A<:AbstractArray{T,1}}
    t, v = s

    if p.current_phase == MaxP
        u = p.train.U̅(v)
    elseif p.current_phase == MaxB
        u = p.train.U̲(v)
    end

    ds1 = 1/v
    ds2 = (u - OptimalTrainControl.r(train, v) + OptimalTrainControl.g(p.track, x)) / v
    SA[ds1, ds2]
end

# choice
V = 10.
ρ = 0.3
# consequence
W = find_zero(v -> -ψ(V) + ρ*ψ(v), 1.)

train = Train(;
        U̅ = v -> 3/max(5., v),
        U̲ = v -> -3/max(5., v),
        r = (1e-2, 0., 1.5e-5),
        ρ
    )

track = Track(;
    length = 10e3,
    x_gradient = [0., 1e3, 2e3, 3e3],
    gradient = [-1e-3, 1.5e-3, -1e-3, -2e-3]
)

track2 = Track(;
    length = 5e3,
    x_gradient = [0., 2e3, 3e3],
    gradient = [0., 35/1e3, 0.]
)

prob = EETCProblem(;
    train,
    track,
    current_phase = MaxP,
    initial_speed = V,
    T = 1e3
)

# low speed termination (0.5 m/s)
lowspeed_cond(s, x, int) = s[2] - 0.5
lowspeed_aff(int) = terminate!(int)
lowspeed_cb = ContinuousCallback(lowspeed_cond, lowspeed_aff)

callbacks = CallbackSet(lowspeed_cb)

xspan = (1205., 10.)
s0 = SA[0., V]
odeprob = ODEProblem(_odefun, s0, xspan, prob;
    tstops = track.x_gradient)

odesol = OrdinaryDiffEq.solve(odeprob, Tsit5();
    callback = callbacks,
    d_discontinuities = track.x_gradient)
plot(odesol.t, odesol[2,:])

η = similar(odesol.t)

Es = [E(V, V)]
for k in reverse(eachindex(odesol.t))
    v = odesol[2,k]
    x = odesol.t[k]
    
    η[k] = (E(V, v) - last(Es)) / (train.U̅(v) - r(v) + g(track, x))

    if x in track.x_gradient
        push!(Es, last(Es) - η[k] * (g(track, x+1.) - g(track, x - 1.)))
    end
end

plot(odesol.t, η)

##

function _odefun(s::A, p::EETCProblem, x::T) where {T<:Real, A<:AbstractArray{T,1}}
    t, v = s

    if p.current_phase == MaxP
        u = p.train.U̅(v)
    elseif p.current_phase == MaxB
        u = p.train.U̲(v)
    end

    ds1 = 1/v
    ds2 = (u - OptimalTrainControl.r(train, v) + OptimalTrainControl.g(p.track, x)) / v
    SA[ds1, ds2]
end

prob = EETCProblem(;
    train,
    track = track2,
    current_phase = MaxP,
    initial_speed = V,
    T = 1e3
)

xspan = (1943.645, 3100.)
s0 = SA[0., V]
odeprob = ODEProblem(_odefun, s0, xspan, prob;
    tstops = track2.x_gradient)

# low speed termination (0.5 m/s)
lowspeed_cond(s, x, int) = s[2] - 0.5
lowspeed_aff(int) = terminate!(int)
lowspeed_cb = ContinuousCallback(lowspeed_cond, lowspeed_aff)

# end after end of the steep section when hitting V

targetspeed_cond(s, x, int) = s[2] - V
targetspeed_aff(int) = terminate!(int)
targetspeed_cb = ContinuousCallback(targetspeed_cond, targetspeed_aff; affect_neg! = nothing)

callbacks = CallbackSet(lowspeed_cb, targetspeed_cb)

odesol = OrdinaryDiffEq.solve(odeprob, Tsit5();
    callback = callbacks,
    d_discontinuities = track2.x_gradient,
    dtmax = 10.)
plot(odesol.t, odesol[2,:])

η = similar(odesol.t)

Es = [-E(V, V)]
for k in eachindex(odesol.t)
    v = odesol[2,k]
    x = odesol.t[k]

    if x in track2.x_gradient
        # @show x
        # η[k] = (E(V, v) + last(Es)) / (train.U̅(v) - OptimalTrainControl.r(train, v) + g(track2, x-0.1))
        # @show η[k]
        push!(Es, (g(track2, x+1.0) - g(track2, x-1.0)) * η[k-1] + last(Es))
        # @show (E(V, v) + last(Es)) / (train.U̅(v) - OptimalTrainControl.r(train, v) + g(track2, x))
        η[k] = (E(V, v) + last(Es)) / (train.U̅(v) - OptimalTrainControl.r(train, v) + g(track2, x))
    else
        η[k] = (E(V, v) + last(Es)) / (train.U̅(v) - OptimalTrainControl.r(train, v) + g(track2, x))
    end
end
@show η[end]
plot(odesol.t, η)

plot(η, odesol[2,:])
xlims!(-5e-3, 5e-3)
ylims!(0, 12)

##

# What I need:
#   - callback for transition from MaxP to Coast and vice versa with _neg
#   - callback for transition from Coast to MaxB and vice versa with _neg
#   - callback to automatically push! next E at the points of grade change
