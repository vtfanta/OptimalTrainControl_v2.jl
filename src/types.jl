using OrdinaryDiffEq

export Mode, Train, Track, OTCSolution, TOTCProblem

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

@kwdef mutable struct Track{T<:Real}
    length::T
    altitude::T = 0
    x_gradient::Union{Nothing,Vector{T}} = nothing
    gradient::Union{Nothing,Vector{T}} = nothing
    x_speedlimit::Union{Nothing,Vector{T}} = nothing
    speedlimit::Union{Nothing,Vector{T}} = nothing
    x_segments::Union{Nothing,Vector{Any}} = nothing

    # Track(l, a, xg, g, xsl, sl) = new{T}(l::T, a, sg, g, xsl, sl, nothing)
end

Track(l, a, xg, g, xsl, sl) = Track(l, a, xg, g, xsl, sl, nothing)

# To allow broadcasting
Base.broadcastable(t::Track) = Ref(t)

struct OTCSolution{T<:Real}
    odesol::OrdinaryDiffEq.ODESolution
    x_phases::Vector{T}
    phases::Vector{Mode}
    control::Function
end

# stands for time-optimal train control
@kwdef mutable struct TOTCProblem{T,S,U}
    train::Train{T,S}
    track::Track{U}
    current_phase::Mode
end

TOTCProblem(train, track) = TOTCProblem(train, track, MaxP)