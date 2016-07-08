

module FMT

using PyPlot

include("fmt_types.jl")
# TODO parametrize this
function get_cost(pt1,pt2)
    return (norm(pt1-pt2))
end

# returns true if x is in goal region
# Goal_pt is always the last point in the planner
function is_goal_pt(x, P::FMTPlanner)
    return (norm(x - P.points[:,(P.params.num_pts-1)]) < P.params.goal_rad)
end

# Returns neighborhood filtered by B
# Both neighbor indices and B are sorted, so can speed up
function filter(N::Neighborhood, set2)
    L1 = length(N.inds); L2 = length(set2)
    set_inds   = Vector{Int64}(L1)
    ind_new    = 1; ind1 = 1; ind2 = 1;
    while(ind1 <= L1 && ind2 <= L2)
        if(N.inds[ind1] == set2[ind2])
            # Add to intersection (only if not in set!)
            set_inds[ind_new] = ind1
            ind_new+=1; ind1+=1; ind2+=1
        elseif(N.inds[ind1] < set2[ind2])
            #Increment ind1
            ind1+=1
        elseif(N.inds[ind1] > set2[ind2])
            # Increment ind2
            ind2+=1
        end
    end
    ind_new -=1
    return Neighborhood(N.inds[set_inds[1:ind_new]], N.ds[set_inds[1:ind_new]])
end

function is_sorted(set)
    L = length(set)
    for i = 2:L
        if(set[i-1] > set[i])
            return false
        end
    end
    return true
end

# Adds elements from second collection to first. Assumes that the first is sorted
function add_elements!(set1, set2)

    ind1 = 1; ind2 = 1;
    L1 = length(set1); L2 = length(set2)
    new_set = Int64[];
    # Interleave sets
    while(ind1 <= L1 || ind2 <= L2)
        if(ind1 <= L1 && ind2 <= L2)
            if(set1[ind1] <= set2[ind2])
                push!(new_set, set1[ind1])
                if(set1[ind1] == set2[ind2])
                    ind2+=1
                end
                ind1+=1
            else
                push!(new_set, set2[ind2])
                ind2+=1
            end
        elseif(ind1 <= L1)
            push!(new_set, set1[ind1])
            ind1+=1
        elseif(ind2 <= L2)
            push!(new_set, set2[ind2])
            ind2+=1
        end
    end
    set1 = new_set
    return new_set
end

# Removes element from sorted set
function remove_element(const_element, const_set)
    set = deepcopy(const_set)
    element = deepcopy(const_element)
    L = length(set)
    # binary search for element
    curr = round(Int, L/2) 
    old_curr = curr

    # check edge cases
    if(set[1] == element)
        splice!(set, 1)
        return set
    end
    if(set[L] == element)
        splice!(set, L)
        return set
    end
    LB = 1
    UB = L
 
    while(LB+1 < UB)
        if(set[curr] == element)
            splice!(set, curr)
            return set
        elseif(set[curr] < element)
            LB = curr+1
            curr = round(Int, (UB-LB)/2+LB)
        else
            UB = curr-1 
            curr = round(Int, (UB-LB)/2+LB)
        end 
    end
    if(set[LB] == element)
        splice!(set, LB)
        return set
    elseif(set[UB]==element)
        splice!(set,UB)
        return set
    else
        return set
    end
end
# Removes element from sorted set
function remove_element!(element, set)
    L = length(set)
    # binary search for element
    curr = round(Int, L/2) 
    old_curr = curr

    # check edge cases
    if(set[1] == element)
        splice!(set, 1)
        return set
    end
    if(set[L] == element)
        splice!(set, L)
        return set
    end
    LB = 1
    UB = L
 
    while(LB+1 < UB)
        if(set[curr] == element)
            splice!(set, curr)
            return set
        elseif(set[curr] < element)
            LB = curr+1
            curr = round(Int, (UB-LB)/2+LB)
        else
            UB = curr-1 
            curr = round(Int, (UB-LB)/2+LB)
        end 
    end
    if(set[LB] == element)
        splice!(set, LB)
        return set
    elseif(set[UB]==element)
        splice!(set,UB)
        return set
    else
        return set
    end
end

# Handle offline computations - sampling, neighborhood construction and cost evaluations 
function precompute(params::FMTParam)
    # sample points (steering should go here eventually)
    points = rand(2,params.num_pts)
    # Scale points by arena size
    points[1,:] *= params.X_lim-1
    points[2,:] *= params.Y_lim-1
    points[1,:] += 1
    points[2,:] += 1
    println("Smallest point value is ", minimum(points))
    # points are in the box [1,Xlim;1,Ylim]

    # Compute neighborhoods
    println("Computing neighborhoods of ", params.num_pts, " points")
    neighborhoods = Vector{Neighborhood}(params.num_pts)
    for pt1 = 1:(params.num_pts-2)
        inds = []
        costs = [] 
        for pt2 = 1:(params.num_pts-2)
            # TODO replace this with appropriate function (knn for ex)
            dist = get_cost(points[:,pt2], points[:,pt1])
            if(dist <= params.radius)
                # TODO replace with appropriate cost function
                pt2_cost = dist
                if(isempty(inds))
                    inds = [pt2]
                    costs = [pt2_cost]
                else
                    inds = [inds;pt2]
                    costs = [costs; pt2_cost]
                end
            end 
        end
        neighborhoods[pt1] = Neighborhood(inds, costs)
    end
    # Start and goal initialization
    neighborhoods[params.num_pts-1] = Neighborhood([params.num_pts-1],[0.])
    neighborhoods[params.num_pts] = Neighborhood([params.num_pts],[0.])
    return FMTPlanner(params, points, neighborhoods)
end




# Based off of Ed Schmerling's implementation at github.com/schmrlng/MotionPlanning.jl
# Takes in initialized FMTPlanner
function fmtstar(start_pt::Vector{Float64}, goal_pt::Vector{Float64}, map::Array{Int64}, P::FMTPlanner)
    # Add goal and start to planner TODO efficient implementation?
    N = P.params.num_pts
    P.points[:,N]     = deepcopy(start_pt)
    P.points[:,N-1]   = deepcopy(goal_pt)
    start_inds = Int64[]; start_costs = Float64[];
    goal_inds  = Int64[]; goal_costs  = Float64[];
    
    for pt = 1:N
        dist_start = get_cost(P.points[:,pt], P.points[:,N])
        dist_goal  = get_cost(P.points[:,pt], P.points[:,N-1])
        if(dist_goal <= P.params.radius && !collision_check(P.points[:,N-1], P.points[:,pt], map))
            push!(goal_inds, pt)
            push!(goal_costs, dist_goal)
            push!(P.neighborhoods[pt].inds,N-1);
            push!(P.neighborhoods[pt].ds,dist_goal)
        end
        if(dist_start <= P.params.radius && !collision_check(P.points[:,N], P.points[:,pt], map))
            push!(start_inds, pt)
            push!(start_costs, dist_start)
            push!(P.neighborhoods[pt].inds,N);
            push!(P.neighborhoods[pt].ds,dist_start)
        end
    end
    P.neighborhoods[N] = Neighborhood(start_inds, start_costs)
    P.neighborhoods[N-1] = Neighborhood(goal_inds, goal_costs)
    # Containers for tracking points
    Vindex = collect(1:N)
    Windex = collect(1:(N-1))

    # Initialize heap
    HHeap = Collections.PriorityQueue([N],[0.]) 
    z = Collections.dequeue!(HHeap)
    A = zeros(Int, N)   # Container to track paths
    H = [z]             # Contains frontiers
    C = zeros(Float64,N)# Tracks costs to come 

    H_sorted = true;

    while ~is_goal_pt(P.points[:,z], P)
        remove_element!(z, Windex)
        H_new = Int64[]
        X_near = filter(P.neighborhoods[z], Windex)
        for x in X_near.inds
            l0 = length(H)
            W_tmp = remove_element(x, H)
            Y_near = filter(P.neighborhoods[x], W_tmp)
            if(!isempty(Y_near.inds))
                c_min, y_ind = findmin(C[Y_near.inds] + Y_near.ds)
                # Check for collision
                if(!collision_check(P.points[:,x],P.points[:,Y_near.inds[y_ind]], map))
                    A[x] = Y_near.inds[y_ind] # Add parent
                    C[x] = c_min              # Cost to go
                    push!(H_new, x)           # Mark as visited
                    HHeap[x] = c_min          # Add to heap
                    remove_element!(x,Windex) # Remove from candidates list
                end
            end
        end
        H = add_elements!(H, H_new) # Update `active' list
        remove_element!(z,H)    # Remove z

        if(!isempty(HHeap))
            z = Collections.dequeue!(HHeap)
        else
            # Try connecting to goal, in case this is a sparse sampling issue
            if(!collision_check(P.points[:,z],P.points[:,N-1],map))
                A[N-1] = z
                C[N-1] = C[z] + get_cost(P.points[:,z], P.points[:,N-1])
                z = N-1
            else
        #        println("Failed to find goal")
                C[z] = Inf
            end
            break
        end
    end 
    sol = [z]
    while(sol[1]!= N)
        unshift!(sol,A[sol[1]])
    end

    if(length(sol)==1)
        C[z] = Inf
    end

    # Clean up: (pop in the reverse order that you pushed)
    for n = N:-1:(N-1)
        for pt in P.neighborhoods[n].inds
            if(pt == n)
            else
                # always the last point added
                pop!(P.neighborhoods[pt].inds)
                pop!(P.neighborhoods[pt].ds)
            end
        end
    end
    P.neighborhoods[N] = Neighborhood([N], [0.])
    P.neighborhoods[N-1] = Neighborhood([N-1],[0.])

    i = 2
    while i < length(sol)
        if(!collision_check(P.points[:,sol[i-1]], P.points[:,sol[i+1]], map)) # Means this point is irrelevant
            splice!(sol, i);
        else
            i+=1
        end
    end

    return sol, C[z], H 
end


# Check collisions
function collision_check(loc1,loc2,map)
    n1 = round(Int, loc1)
    n2 = round(Int, loc2)
  #  println("Checking points $pt1, $pt2")
    if(n1 == n2)
        return (map[n1[1],n1[2]]==1)
    end 
    if(map[n1[1],n1[2]] == 1 || map[n2[1],n2[2]] == 1)
        return true
    end 
    
    dy = n2[2] - n1[2]; dx = n2[1] - n1[1]
    search_axis = [dx,dy]./(max(abs(dx),abs(dy)))

    # Test for collision
    test_pt = n1
    cnt = 0
    while(true)
    # test condition depending on deltas
    if(dy >= 0)
        if(dx >= 0)
            # Traveling in positive directions
            condition = (test_pt[1] <= n2[1] && test_pt[2] <= n2[2])
        else
            # Traveling in +y -x direction
            condition = (test_pt[1] >= n2[1] && test_pt[2] <= n2[2])
        end
    else
        if(dx >= 0)
            # Traveling in -y +x direction
            condition = (test_pt[1] <= n2[1] && test_pt[2] >= n2[2])
        else
            # Traveling in -y -x direction
            condition = (test_pt[1] >= n2[1] && test_pt[2] >= n2[2])
        end
    end
    if(condition==false)
        break
    end

    cnt+=1
    if map[round(Int,test_pt[1]),round(Int,test_pt[2])] == 1
#        println("Collision found!")
        return true
    end

    # Check for split lines (3 cases):
    if(mod(test_pt[1],1.0) == 0.5)
        if map[round(Int,test_pt[1]-0.1),round(Int,test_pt[2])] == 1
           return true
        end
        # double split
        if(mod(test_pt[2],1.0) == 0.5)
            if map[round(Int,test_pt[1]-0.1),round(Int,test_pt[2]-0.1)] == 1
               return true
            end
        end
    end
    if(mod(test_pt[2],1.0) == 0.5)
        if map[round(Int,test_pt[1]),round(Int,test_pt[2]-0.1)] == 1
           return true
        end
    end
    # Increment along search axis
        test_pt += search_axis
    end
    return false
end



function test_fmt()
    println("Initializing FMT")
    # initialize parameters
    params = FMTParam(100,400,1000,14, 14)
    @time P = precompute(params) 

    start_pt = [1.,1.]
    goal_pt  = [params.X_lim*0.99, params.Y_lim*0.99]
    println("Planning from $start_pt to $goal_pt")

    map = zeros(params.X_lim,params.Y_lim);
    for i = 1:params.X_lim
        for j = 1:params.Y_lim
            if(i > 0.05*params.X_lim && i < 0.9*params.X_lim)
                if(j > 0.05*params.Y_lim && j < 0.9*params.Y_lim)
                    map[i,j] = 1
                end
            end
        end
    end
    map = round(Int, map)

    println("Finding path")
    @time path, cost, touched = fmtstar(start_pt, goal_pt, map, P)
    println("Path is $path with cost $cost")

    @time path, cost, touched = fmtstar(start_pt, goal_pt, map, P)
    println("Path is $path with cost $cost")

    @time path, cost, touched = fmtstar(start_pt, goal_pt, map, P)
    println("Path is $path with cost $cost")

# Visualize paths:
    figure(1); clf();
    scatter(P.points[1,:], P.points[2,:])

    # Plot debugging stuff
    if(cost == -1)

    scatter(P.points[1,touched], P.points[2,touched], color=:red)

    for pt = 1:params.num_pts
        for pt2 in P.neighborhoods[pt].inds
            if(pt2 > pt && !collision_check(P.points[:,pt], P.points[:,pt2], map))
                x = [P.points[1,pt], P.points[1,pt2]]
                y = [P.points[2,pt], P.points[2,pt2]]
                plot(x,y, color=:black)
            end
        end
    end
    end

    plot(P.points[1,path]', P.points[2,path]')
    imshow(map',interpolation="none")
    return P, path
end

end
