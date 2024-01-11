using OrdinaryDiffEq

export Mode, Train, Track, OTCSolution, TOTCProblem, EETCProblem
export MaxP, HoldP, HoldR, Coast, MaxB
export Port

@enum Mode begin
    MaxP = 0
    HoldP = 1
    HoldR = 2
    Coast = 4
    MaxB = 8
end    

@kwdef struct Train{T<:Real,S<:Real}
    U̅::Function
    U̲::Function
    r::NTuple{3,T}
    ρ::S
end

Train(U̅, U̲, r) = Train(U̅, U̲, r, 0)

# To allow broadcasting
Base.broadcastable(t::Train) = Ref(t)

@kwdef mutable struct Track{T<:Real, G<:Union{Nothing, Vector{T}},
    S<:Union{Nothing, Vector{T}}}
    length::T
    altitude::T = 0.
    x_gradient::G = nothing
    gradient::G = nothing
    x_speedlimit::S = nothing
    speedlimit::S = nothing
    x_segments::Union{Nothing,Vector{Any}} = nothing
end

Track(l, a, xg, g, xsl, sl) = Track(l, a, xg, g, xsl, sl, nothing)

# To allow broadcasting
Base.broadcastable(t::Track) = Ref(t)

struct OTCSolution{T<:Real,S<:Union{Nothing,Vector{<:Real}}}
    odesol::OrdinaryDiffEq.ODESolution
    x_phases::Vector{T}
    phases::Vector{Mode}
    control::Function
    η::S
end
OTCSolution(sol, x_phases, phases, control) = OTCSolution(sol, x_phases, phases, control, nothing)

# stands for time-optimal train control
@kwdef mutable struct TOTCProblem{T,S,U,V<:AbstractFloat}
    train::Train{T,S}
    track::Track{U}
    current_phase::Mode = MaxP
    initial_speed::V = 1.
end

TOTCProblem(train, track) = TOTCProblem(train, track, MaxP, 1.)
TOTCProblem(train, track, mode) = TOTCProblem(train, track, mode, 1.)

# stands for energy-efficient train control
@kwdef mutable struct EETCProblem{TV,S,U,TG,TS,V<:AbstractFloat,W<:AbstractFloat}
    T::V
    train::Train{TV,S}
    track::Track{U,TG,TS}
    current_phase::Mode = MaxP
    initial_speed::V = 1.
    Es::Vector{W} = Float64[]
end

EETCProblem(T, train, track, mode) = EETCProblem(T, train, track, mode, 1., Float64[])
EETCProblem(T, train, track) = EETCProblem(T, train, track, MaxP)

struct Port{T<:AbstractFloat}
    start::T
    finish::T
    mode::Mode
    speed::T
end