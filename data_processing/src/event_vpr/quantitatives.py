import numpy as np
from tqdm.auto import tqdm
import os

def get_translation_error(tr1, tr2):
    error = tr1 - tr2
    return np.sqrt(error[1]**2 + error[2]**2)

# Splits predictions into TP, FP, FN and maintains a flag array called 'verdict'
def get_verdict(ref_odom, qry_odom, predictions, distances, distanceThreshold=2.0):
    # Find the top prediction for each query
    top_predictions = predictions[:,0]
    top_distances = distances[:,0]

    # Check with a threshold, filter out false negatives(-1)
    verdict = np.zeros(len(top_distances))
    verdict[np.where(top_distances > distanceThreshold)] = -1

    # Find the indices of true positives(+1) and false positives(0)
    for qIx, pred in enumerate(top_predictions):
        # if verdict[qIx] == 0 and pred in gt[qIx]:
        if ((verdict[qIx] == 0) and get_translation_error(ref_odom[pred], qry_odom[qIx]) < 5.0):
            verdict[qIx] = 1
    return verdict, top_predictions

def compute_recall(gt, predictions, numQ, n_values, recall_str='', directory_path=None):
    correct_at_n = np.zeros(len(n_values))
    for qIx, pred in enumerate(predictions):
        for i, n in enumerate(n_values):
            # if in top N then also in top NN, where NN > N
            if np.any(np.in1d(pred[:n], gt[qIx])):
                correct_at_n[i:] += 1
                break
    recall_at_n = correct_at_n / numQ
    all_recalls = {}  # make dict for output
    for i, n in enumerate(n_values):
        all_recalls[n] = recall_at_n[i]
        tqdm.write(
            "====> Recall {}@{}: {:.4f}".format(recall_str, n, recall_at_n[i]))
        with open(os.path.join(directory_path), 'a') as f:
            f.write("{:.4f}\n".format(recall_at_n[i]))
    return all_recalls

def get_odom_distances(ref_odom, qry_odom):
    distances = np.zeros((ref_odom.shape[0], qry_odom.shape[0]))
    for i in range(ref_odom.shape[0]):
        for j in range(qry_odom.shape[0]):
            distances[i, j] = get_translation_error(ref_odom[i], qry_odom[j])
    return distances