import sys
import numpy as np
import argparse
import associate
from scipy.spatial.distance import cdist

def align(model, data):
    """Align two trajectories using the method of Horn (closed-form)."""
    np.set_printoptions(precision=3, suppress=True)
    model_zerocentered = model - model.mean(axis=1)
    data_zerocentered = data - data.mean(axis=1)
    
    W = np.zeros((3, 3))
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:, column], data_zerocentered[:, column])
    U, d, Vh = np.linalg.svd(W.transpose())
    S = np.matrix(np.identity(3))
    if np.linalg.det(U) * np.linalg.det(Vh) < 0:
        S[2, 2] = -1
    rot = U * S * Vh

    rotmodel = rot * model_zerocentered
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
        dots += np.dot(data_zerocentered[:, column].transpose(), rotmodel[:, column])
        normi = np.linalg.norm(model_zerocentered[:, column])
        norms += normi * normi

    s = float(dots.item() / norms.item())  # Use .item() to avoid deprecation warning
    
    trans = data[:, 0] - s * rot * model[:, 0]
    
    model_aligned = s * rot * model + trans
    alignment_error = model_aligned - data
    trans_error = np.sqrt(np.sum(np.multiply(alignment_error, alignment_error), axis=0)).A1
        
    return rot, trans, trans_error, s

def plot_traj(ax, stamps, traj, style, color, label):
    """
    Plot a trajectory using matplotlib. 
    
    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    """
    stamps = sorted(stamps)  # Convert to sorted list
    interval = np.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    
    print(f"Debug: Number of stamps: {len(stamps)}")
    print(f"Debug: Shape of traj: {traj.shape}")
    
    for i in range(min(len(stamps), len(traj))):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label,linewidth=0.5)
            label=""
            x=[]
            y=[]
        last = stamps[i]
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label,linewidth=0.5)

def compute_distances(first_xyz, second_xyz_aligned):
    """
    Compute the shortest distance from each point in second_xyz_aligned
    to the trajectory defined by first_xyz.
    """
    distances = []
    for i in range(second_xyz_aligned.shape[1]):
        point = second_xyz_aligned[:, i]
        # Compute distances to all points in first_xyz
        dists = cdist(point.T.A, first_xyz.T.A)
        # Find the minimum distance
        min_dist = np.min(dists)
        distances.append(min_dist)
    return np.array(distances)

if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('second_file', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)',default=1.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 10000000 ns)',default=20000000)
    parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    args = parser.parse_args()

    first_list = associate.read_file_list(args.first_file, False)
    second_list = associate.read_file_list(args.second_file, False)

    matches = associate.associate(first_list, second_list,float(args.offset),float(args.max_difference))    
    if len(matches)<2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")

    first_xyz = np.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
    second_xyz = np.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for a,b in matches]).transpose()
    
    print(f"Debug: Number of matches: {len(matches)}")
    print(f"Debug: Shape of first_xyz: {first_xyz.shape}")
    print(f"Debug: Shape of second_xyz: {second_xyz.shape}")
    
    rot, trans, trans_error, scale = align(second_xyz, first_xyz)
    
    second_xyz_aligned = scale * rot * second_xyz + trans
    
    print(f"Debug: Shape of second_xyz_aligned: {second_xyz_aligned.shape}")
    
    # Compute distances between aligned estimated trajectory and ground truth
    distances = compute_distances(first_xyz, second_xyz_aligned)
    
    # Calculate mean and standard deviation
    mean_distance = np.mean(distances)
    std_distance = np.std(distances)
    
    print(f"Mean distance between estimated and ground truth: {mean_distance:.4f} m")
    print(f"Standard deviation of distances: {std_distance:.4f} m")

    if args.verbose:
        print("compared_pose_pairs %d pairs"%(len(trans_error)))
        print("absolute_translational_error.rmse %f m"%np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)))
        print("absolute_translational_error.mean %f m"%np.mean(trans_error))
        print("absolute_translational_error.median %f m"%np.median(trans_error))
        print("absolute_translational_error.std %f m"%np.std(trans_error))
        print("absolute_translational_error.min %f m"%np.min(trans_error))
        print("absolute_translational_error.max %f m"%np.max(trans_error))
    else:
        print("%f,%f" % (np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)), scale))
    
    if args.save_associations:
        file = open(args.save_associations,"w")
        file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A)]))
        file.close()
        
    if args.save:
        file = open(args.save,"w")
        file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_list.keys(),second_xyz_aligned.transpose().A)]))
        file.close()

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111)
        
        print(f"Debug: Number of first_list keys: {len(first_list.keys())}")
        print(f"Debug: Number of second_list keys: {len(second_list.keys())}")
        
        plot_traj(ax, list(first_list.keys()), first_xyz.T.A, '-', "black", "ground truth")
        plot_traj(ax, list(second_list.keys()), second_xyz_aligned.T.A, '-', "blue", "estimated")
        
        ax.legend(loc='upper left', fontsize='small')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.axis('equal')
        plt.title(f'ATE: mean={mean_distance:.4f}m, std={std_distance:.4f}m')
        plt.tight_layout()
        plt.savefig(args.plot, format="pdf", dpi=300)

    #if args.plot:
        #import matplotlib
        #matplotlib.use('Agg')
        #import matplotlib.pyplot as plt
        
        #fig = plt.figure(figsize=(8, 8))
        #ax = fig.add_subplot(111)
        
        # Plot ground truth (transparent black)
        #ax.plot(first_xyz[0, :].A[0], first_xyz[1, :].A[0], '-', color="black", alpha=0, label="ground truth", linewidth=1.5)
        
        # Plot aligned estimated trajectory (red instead of blue)
        #ax.plot(second_xyz_aligned[0, :].A[0], second_xyz_aligned[1, :].A[0], '-', color="red", label="estimated", linewidth=1.5)
        
        #ax.legend(loc='upper left', fontsize='small')
        #ax.set_xlabel('x [m]')
        #ax.set_ylabel('y [m]')
        #plt.axis('equal')
        #plt.tight_layout()
        #plt.savefig(args.plot, format="pdf", dpi=300)