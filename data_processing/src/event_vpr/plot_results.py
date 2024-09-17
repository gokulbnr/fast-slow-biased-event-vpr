import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
from event_vpr.quantitatives import get_verdict


def get_plot_distance_matrix(distances):
    ax = sns.heatmap(distances, cmap='winter', cbar=True)
    plt.xlabel('Queried indices')
    plt.ylabel('Returned indices')
    plt.title('Similarity Matrix')
    ax.set_aspect('equal')
    plt.xticks(rotation=60, fontsize=7)
    plt.yticks(rotation=30, fontsize=7)
    return plt


def get_plot_odom(*args):
    plt.figure()
    sns.set(style="whitegrid")
    for data in args:
        sns.scatterplot(x=data[:,1],y=data[:,2], s=1)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Trajectory')
    plt.grid(True)
    plt.axis('square')
    return plt


def get_plot_prediction_indices(pred):
    sns.set_theme(style="darkgrid")
    ax = sns.scatterplot(x=range(pred.shape[0]), y=pred[:,0], s=2, color='blue')
    plt.xlim(0, len(pred[:,0]))
    plt.ylim(0, len(pred[0,:]))
    ax.invert_yaxis()
    plt.xlabel('Query Indices')
    plt.ylabel('Predicted Indices')
    plt.title('Prediction Vs Query Index Plot')
    ax.set_aspect('equal')
    ax.set_xticks(np.arange(0, len(pred[:,0]), 100))
    ax.set_yticks(np.arange(0, max(pred[:,0]), 100))
    plt.xticks(rotation=60, fontsize=7)
    plt.yticks(rotation=30, fontsize=7)
    return plt

def get_plot_prediction_and_gt_indices(pred, gt):
    print(pred.shape, len(gt))
    sns.set_theme(style="darkgrid")
    ax = sns.scatterplot(x=range(pred.shape[0]), y=pred[:,0], s=2, color='blue')
    ax = sns.scatterplot(x=range(len(gt)), y=gt[:][0], s=2, color='red')
    plt.xlim(0, len(pred[:,0]))
    plt.ylim(0, len(pred[0,:]))
    ax.invert_yaxis()
    plt.xlabel('Query Indices')
    plt.ylabel('Predicted Indices')
    plt.title('Prediction Vs Query Index Plot')
    ax.set_aspect('equal')
    ax.set_xticks(np.arange(0, len(pred[:,0]), 100))
    ax.set_yticks(np.arange(0, max(pred[:,0]), 100))
    plt.xticks(rotation=60, fontsize=7)
    plt.yticks(rotation=30, fontsize=7)
    return plt


def get_plot_predictions_on_distance(ref_odom, qry_odom,  pred, distances):

    query_displacements = np.diff(qry_odom, axis=0)
    query_displacements = np.sqrt(query_displacements[:,1]**2 + query_displacements[:,2]**2)
    current_norm = np.linalg.norm(query_displacements)
    query_displacements = distances.shape[0]-1 -1 * (query_displacements * (max(pred[:,0])/current_norm))

    reference_displacements = np.diff(ref_odom, axis=0)
    reference_displacements = np.sqrt(reference_displacements[:,1]**2 + reference_displacements[:,2]**2)
    current_norm = np.linalg.norm(reference_displacements)
    reference_displacements = (reference_displacements * (max(pred[:,0])/current_norm))

    ax = sns.heatmap(distances, cmap='winter', cbar=True)
    ax = sns.scatterplot(x=range(pred.shape[0]), y=pred[:,0], s=2, color='red', label='predictions')
    ax = sns.scatterplot(x=range(query_displacements.shape[0]), y=query_displacements, s=2, color='black', edgecolor='black', label='qry_vel')
    ax = sns.scatterplot(x=reference_displacements, y=range(reference_displacements.shape[0]), s=2, color='white', edgecolor='white', label='ref_vel')
    plt.title('Top Predictions on Distance Matrix')
    plt.xlabel('Queried indices')
    plt.ylabel('Predicted indices')
    ax.set_xticks(np.arange(0, len(pred[:,0]), 100))
    ax.set_yticks(np.arange(0, max(pred[:,0]), 100))
    plt.xticks(rotation=60, fontsize=7)
    plt.yticks(rotation=30, fontsize=7)
    ax.set_aspect('equal')
    legend = plt.legend(loc='center left', bbox_to_anchor=(-0.37, 1.0))
    for line in legend.get_texts():
        line.set_fontsize(7)
    for line in legend.legend_handles:
        line._sizes = [30]
    return plt


def get_plot_precision_recall(ref_odom, qry_odom, predictions, distances):
    precision_list = []
    recall_list = []
    min_val = np.min(distances[:,0])
    max_val = np.max(distances[:,0])
    
    for distanceThreshold in np.arange(min_val, max_val+0.1, (max_val-min_val)/99):
        verdict, _ = get_verdict(ref_odom, qry_odom, predictions, distances, distanceThreshold)
        numTP = np.where(verdict == 1)[0].shape[0]
        numFP = np.where(verdict == 0)[0].shape[0]
        numFN = np.where(verdict == -1)[0].shape[0]
        precision = numTP/(numTP+numFP)
        recall = numTP/(numTP+numFN)
        precision_list.append(precision)
        recall_list.append(recall)

    print("Maximum Recall@99%Precision: ", recall_list[np.max(np.where(np.array(precision_list) >= 0.99))])

    sns.set_theme(style="darkgrid")
    sns.lineplot(x=recall_list, y=precision_list, color='blue')
    sns.scatterplot(x=recall_list, y=precision_list, color='blue', marker='o', s=10)
    plt.ylim(-0.01,1.01)
    plt.xlim(-0.01,1.01)
    plt.xlabel('Recall')
    plt.ylabel('Precision')
    plt.title('Precision-Recall Curve')
    return plt