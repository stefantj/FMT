
# Type that holds neighborhoods of a point
type Neighborhood{T<:FloatingPoint}
    inds::Vector{Int} # Indices of neighbors
    ds::Vector{T}     # Cost of successful connection
end

# Always assume in box 0,1?
type FMTParam
    X_lim::Int64
    Y_lim::Int64
    num_pts::Int64
    radius::Float64
    goal_rad::Float64
end

# Type that holds precomputed information for the path planner
type FMTPlanner
    params::FMTParam                    # Parameter object
    points::Array{Float64,2}            # Vector of points TODO parametrize dimension
    neighborhoods::Vector{Neighborhood} # Vector of all neighborhoods of points
end
