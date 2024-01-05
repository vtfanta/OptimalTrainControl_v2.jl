using OrdinaryDiffEq
using Roots
using StaticArrays

export solve

function _solve(p::EETCProblem{TV,S,U,Nothing,Nothing,VS}, V::A) where {TV, S, U, VS, A<:AbstractFloat}
    segment = Port(0., p.track.length, HoldP, V)
end

# solving EETC on a flat track without speed limits
function solve(p::EETCProblem{TV,S,U,Nothing,Nothing,VS}) where {TV,S,U,VS}
    println("$(typeof(p.track.gradient))")
    # On a flat track, the mode sequence goes as MaxP -> (HoldP) -> Coast -> MaxB

    # first guess of the holding speed
    V = p.track.length / p.T 
    
    _solve(p, V)
end