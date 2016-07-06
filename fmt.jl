using PyPlot

#include("fmt_types.jl")
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

# Returns index of element if found. If not found, returns negative of index where item would be. 
function binary_search(element, set, LB_0, UB_0)
    if(set[LB_0] == element)
        return LB_0
    elseif(set[UB_0] == element)
        return UB_0
    else
        LB = LB_0
        UB = UB_0

        while(LB+1 < UB)
            curr = round(Int, (UB-LB)/2+LB)
            if(set[curr] == element)
                return curr
            elseif(set[curr] < element)
                LB = curr+1
            else
                UB = curr-1
            end
        end
        if(set[LB] == element)
            return LB
        elseif(set[UB] == element)
            return UB
        else
            return -UB
        end
    end
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
    # Compute neighborhoods
    println("Computing neighborhoods of ", params.num_pts, " points")
    neighborhoods = Vector{Neighborhood}(params.num_pts)
    for pt1 = 1:(params.num_pts-2)
        if(mod(pt1,100)==0)
            println("pt $pt1")
        end
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
    start_inds = []; start_costs = [];
    goal_inds  = []; goal_costs  = [];
    for pt = 1:N
        dist_start = get_cost(P.points[:,pt], P.points[:,N])
        dist_goal  = get_cost(P.points[:,pt], P.points[:,N-1])
        if(dist_start <= P.params.radius)
            if(isempty(start_inds))
                start_inds = [pt]; start_costs = [dist_start];
            else
                push!(start_inds, pt); push!(start_costs, dist_start);
            end
            push!(P.neighborhoods[pt].inds,N);
            push!(P.neighborhoods[pt].ds,dist_start)
        end
        if(dist_goal <= P.params.radius)
            if(isempty(goal_inds))
                goal_inds = [pt]; goal_costs = [dist_goal];
            else
                push!(goal_inds, pt); push!(goal_costs, dist_goal);
            end
            push!(P.neighborhoods[pt].inds,N-1);
            push!(P.neighborhoods[pt].ds,dist_goal)
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
                println("Failed to find goal! ", P.neighborhoods[z])
                println(H)
                C[z] = -1
            end
            break
        end
    end 
    sol = [z]
    while(sol[1]!= N)
        unshift!(sol,A[sol[1]])
    end

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
    # locations are in [0,1]^2, points are in {0,N}^2
    scaler = size(map,2)
    pt1 = round(Int, loc1.*(scaler-1))+1
    pt2 = round(Int, loc2.*(scaler-1))+1
#    println("Checking points $pt1, $pt2")
    
    if(map[pt1[1],pt1[2]] == 1 || map[pt2[1],pt2[2]] == 1)
        return true
    end 
    n1 = zeros(2); n2 = zeros(2)
    if(pt1[2] == pt2[2])
        if(pt1[1] < pt2[1])
            n1[1] = pt1[1]; n1[2] = pt1[2];
            n2[1] = pt2[1]; n2[2] = pt2[2];
        else
            n1[1] = pt2[1]; n1[2] = pt2[2];
            n2[1] = pt1[1]; n2[2] = pt1[2];
        end
    elseif(pt1[2] < pt2[2])
        n1[1] = pt1[1]; n1[2] = pt1[2];
        n2[1] = pt2[1]; n2[2] = pt2[2];
    else
        n1[1] = pt2[1]; n1[2] = pt2[2];
        n2[1] = pt1[1]; n2[2] = pt1[2];
    end

    dy = n2[2] - n1[2]; dx = n2[1] - n1[1]
    if abs(dy) > abs(dx)
        # more steps in y
        if(abs(dx == 0))
            search_axis = [0,1]
        else
            search_axis = [dx/dy,1]
        end
    else
        if(abs(dy==0))
            search_axis = [1,0]
        else
            search_axis = [1,dy/dx]
        end
    end

    # Test for collision
    test_pt = deepcopy(n1)
    while(test_pt[1] <= n2[1] && test_pt[2] <= n2[2])
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
    params = FMTParam(1000,0.12, 0.1)
    @time P = precompute(params) 

    start_pt = [0.,0.]
    goal_pt  = [1.,1.]

    map = zeros(100,100);
    for i = 1:100
        for j = 1:100
            if(i > 5 && i < 90)
                if(j > 5 && j < 90)
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
    scatter(P.points[1,:]*100, P.points[2,:]*100)

    # Plot connections in graph
    if(cost == -1)

    scatter(P.points[1,touched]*100, P.points[2,touched]*100, color=:red)

    for pt = 1:params.num_pts
        for pt2 in P.neighborhoods[pt].inds
            if(pt2 > pt && !collision_check(P.points[:,pt], P.points[:,pt2], map))
                x = [P.points[1,pt]*100, P.points[1,pt2]*100]
                y = [P.points[2,pt]*100, P.points[2,pt2]*100]
                plot(x,y, color=:black)
            end
        end
    end
    end

    plot(P.points[1,path]'*100, P.points[2,path]'*100)
    imshow(map,interpolation="none")
    return P, path
end
