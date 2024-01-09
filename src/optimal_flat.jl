using OrdinaryDiffEq
using Roots

φ(train::Train, v) = v * r(train, v)
ψ(train::Train, v) = (train.r[2] + 2train.r[3]*v) * v^2
E(train::Train, V, v) = ψ(train, V)/v + OptimalTrainControl.r(train, v)

function _odefun_flat(s::A, p::EETCProblem, x::T) where {T<:Real, A<:AbstractArray{T,1}}
    t, v = s

    if p.current_phase == MaxP
        u = p.train.U̅(v)
    elseif p.current_phase == Coast
        u = 0.
    elseif p.current_phase == MaxB
        u = p.train.U̲(v)
    end
    # @show u

    ds1 = 1/v
    ds2 = (u - r(p.train, v)) / v
    SA[ds1, ds2]
end

function _solve(p::EETCProblem{TV,S,U,Nothing,Nothing,VS}, V::A) where {TV, S, U, VS, A<:AbstractFloat}

    ## MaxP phase
    targetspeed_cond(s, x, int) = s[2] - V
    targetspeed_aff(int) = terminate!(int)
    targetspeed_cb = ContinuousCallback(targetspeed_cond, targetspeed_aff; affect_neg! = nothing)

    maxprob = EETCProblem(p.T, p.train, p.track, MaxP)
    s0 = SA[0., p.initial_speed]
    xspan = (0., p.track.length)
    odeprob = ODEProblem(_odefun_flat, s0, xspan, maxprob)
    odesol = OrdinaryDiffEq.solve(odeprob, Tsit5(); callback = targetspeed_cb)
    v1 = odesol[2,:]
    x1 = odesol.t
    η1 = (E.(p.train, V, v1) .- E(p.train, V, V)) ./ (p.train.U̅.(v1) .- r.(p.train, v1))

    ## Coast phase
    L(v) = E(p.train, V, V) * (v - V) + φ(p.train, V)
    v_coast2brake = 0.
    if p.train.ρ ≈ 0
        v_coast2brake = Roots.find_zero(v -> L(v), V)
    elseif p.train.ρ ≈ 1
        v_coast2brake = V
    else
        v_coast2brake = Roots.find_zero(v -> L(v) - p.train.ρ*φ(p.train,v), V)
    end
    @show v_coast2brake
    targetspeed2_cond(s, x, int) = s[2] - v_coast2brake
    targetspeed2_aff(int) = terminate!(int)
    targetspeed2_cb = ContinuousCallback(targetspeed2_cond, targetspeed2_aff)

    coastprob = EETCProblem(p.T, p.train, p.track, Coast)
    s0 = SA[odesol[1,end], V]
    xspan = (x1[end], p.track.length)
    odeprob = ODEProblem(_odefun_flat, s0, xspan, coastprob)
    odesol = OrdinaryDiffEq.solve(odeprob, Tsit5(); callback = targetspeed2_cb)
    v2 = odesol[2,:]
    x2 = odesol.t
    η2 = (E.(p.train, V, v2) .- E(p.train, V, V)) ./ (- r.(p.train, v2))

    ## MaxB phase


    ## construct the solution

end

# solving EETC on a flat track without speed limits
function solve(p::EETCProblem{TV,S,U,Nothing,Nothing,VS}) where {TV,S,U,VS}
    println("$(typeof(p.track.gradient))")
    # On a flat track, the mode sequence goes as MaxP -> (HoldP) -> Coast -> MaxB

    # first guess of the holding speed
    V = p.track.length / p.T 
    
    _solve(p, V)
end