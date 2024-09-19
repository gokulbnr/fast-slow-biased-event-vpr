import cv2
import argparse
import numpy as np
import os
from sklearn.neighbors import NearestNeighbors
from tqdm import tqdm

from event_vpr.plot_results import *
from event_vpr.quantitatives import get_odom_distances, compute_recall

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-I", "--similarity_matrix_filepath", type=str, required=True)
    parser.add_argument("-W", "--save_directory_path", type=str, required=True)
    parser.add_argument("-D", "--image_data_dir_path", type=str)
    parser.add_argument("-E", "--image_data_dir_path_2", type=str)
    args = parser.parse_args()

    directory_path = os.path.join(args.save_directory_path, os.path.basename(args.similarity_matrix_filepath).split('.')[0])
    temp_directory_path = os.path.join(args.save_directory_path, 'temp.txt')
    os.makedirs(directory_path, exist_ok=True)

    experiment_data = np.load(args.similarity_matrix_filepath)
    descriptor_distances = experiment_data['distances']
    reference_odom = experiment_data['reference_odom']
    query_odom = experiment_data['query_odom']

    print(descriptor_distances.shape, reference_odom.shape, query_odom.shape)

    distances = []
    gt = []
    for q in range(len(query_odom)):
        query_xy = query_odom[q,1:3]
        dist_to_qry = []
        for r in range(len(reference_odom)):
            reference_xy = reference_odom[r,1:3]
            distance = np.linalg.norm(query_xy - reference_xy)
            dist_to_qry.append(distance)
        distances.append(dist_to_qry)

    print(np.array(distances).shape)

    gt = np.argsort(np.array(distances), axis=1)
    distances = np.sort(np.array(distances), axis=1)
    print(gt.shape)

    final_gt = []
    for i in range(gt.shape[0]):
        final_gt.append(gt[i][np.where(distances[i] < 5.0)])

    gt_distances = get_odom_distances(reference_odom, query_odom)
    sorted_gt_distances = np.sort(gt_distances.T, axis=1)
    sorted_gt_indices = np.argsort(gt_distances.T, axis=1)
    
    # filtered_gt_distances = gt_distances[np.where(gt_distances <= 5)]
    # sorted_filtered_gt_distances = np.sort(filtered_gt_distances.T, axis=1)
    # sorted_filtered_gt_indices = np.argsort(filtered_gt_distances.T, axis=1)

    # print('A', gt_distances.shape)

    # print('N', filtered_gt_distances.shape)

    sorted_distances = np.sort(descriptor_distances.T, axis=1)
    sorted_indices = np.argsort(descriptor_distances.T, axis=1)
    compute_recall(final_gt, sorted_indices, len(query_odom), [1, 5, 10, 20, 50, 100], recall_str='@1', directory_path=temp_directory_path)

    # print(query_odom[800,1:3])
    # print(reference_odom[final_gt[800][0],1:3])
    # print(final_gt[800])

    pred_index_end = 0
    prev_end = 0
    for i in range(0, len(query_odom), 250):
        try:
            temp_gt = []
            if i+249 < len(query_odom):
                upper_end = 249
            else:
                upper_end = len(query_odom) - i - 1

            pred_index_beg = final_gt[i][0]
            pred_index_end = final_gt[i+upper_end][0]

            temp_gt = []
            for j in range(i,i+upper_end+1):
                mask = np.logical_and((final_gt[j] >= pred_index_beg), (final_gt[j] < pred_index_end+1))
                temp_gt.append(final_gt[j][mask]-pred_index_beg)

            # plt = get_plot_distance_matrix(descriptor_distances.T[i:i+upper_end+1, pred_index_beg:pred_index_end].T)
            # plt.show()
            # plt.clf()

            # plt = get_plot_prediction_indices(np.argsort(descriptor_distances.T[i:i+upper_end+1, pred_index_beg:pred_index_end], axis=1))
            # plt.show()
            # plt.clf()

            # plt.scatter(reference_odom[pred_index_beg:pred_index_end,1], reference_odom[pred_index_beg:pred_index_end,2], c='r', s=5)
            # plt.scatter(query_odom[i:i+upper_end+1,1], query_odom[i:i+upper_end+1,2], c='b', s=1)

            # plt.scatter([reference_odom[pred_index_beg,1],reference_odom[pred_index_end,1]], [reference_odom[pred_index_beg,2], reference_odom[pred_index_end,2]], c='black', s=20)
            # plt.scatter([query_odom[i,1],query_odom[i+upper_end,1]], [query_odom[i,2], query_odom[i+upper_end,2]], c='black', s=20)

            compute_recall(temp_gt, np.argsort(descriptor_distances.T[i:i+upper_end+1, pred_index_beg:pred_index_end], axis=1), upper_end+1, [1], recall_str='@1', directory_path=temp_directory_path)
        
        except Exception as e:
            print('Error')
            continue
    
    # plt.axis('square')
    # plt.show()
    # plt.clf()

    # Similarity Matrix
    plt = get_plot_distance_matrix(descriptor_distances)
    plt.savefig(os.path.join(directory_path, 'distance_matrix.png'))
    plt.clf()

    # Predictions Matrix
    plt = get_plot_prediction_indices(sorted_indices)
    plt.savefig(os.path.join(directory_path, 'predictions.png'))
    plt.clf()

    # Predictions on Similarity Matrix
    plt = get_plot_predictions_on_distance(reference_odom, query_odom, sorted_indices, descriptor_distances)
    plt.savefig(os.path.join(directory_path, 'predictions_on_distance_matrix.png'))
    plt.clf()

    # PR Curve
    plt = get_plot_precision_recall(reference_odom, query_odom, sorted_indices, sorted_distances)
    plt.savefig(os.path.join(directory_path, 'precision_recall.png'))
    plt.clf()


 
    # # Save top matching images
    # ref_file_list = sorted(os.listdir(args.image_data_dir_path))[200:]
    # qry_file_list = sorted(os.listdir(args.image_data_dir_path_2))[200:]
    # for i in tqdm(range(0, len(query_odom))):
    #     query_image = cv2.imread(os.path.join(args.image_data_dir_path_2, str(qry_file_list[i])))
    #     predicted_image = cv2.imread(os.path.join(args.image_data_dir_path, str(ref_file_list[sorted_indices[i][0]])))
    #     gt_image = cv2.imread(os.path.join(args.image_data_dir_path, str(ref_file_list[sorted_gt_indices[i][0]])))
    #     concatenated_image = np.concatenate([query_image, predicted_image, gt_image], axis=1)

    #     fig, ax = plt.subplots(nrows=2, ncols=1)
    #     ax[0].imshow(cv2.cvtColor(concatenated_image, cv2.COLOR_BGR2RGB))
    #     ax[0].set_axis_off()
    #     ax[0].text(query_image.shape[1] // 2, query_image.shape[0] + 30, "Query "+str(i), ha='center')
    #     ax[0].text(query_image.shape[1] + predicted_image.shape[1] // 2, query_image.shape[0] + 30, "Match "+str(sorted_indices[i][0]), ha='center')
    #     ax[0].text(query_image.shape[1] + predicted_image.shape[1] + gt_image.shape[1] // 2, query_image.shape[0] + 30, "Ground Truth "+str(sorted_gt_indices[i][0]), ha='center')

    #     pre_x = reference_odom[np.argmin(descriptor_distances[:,i])][1]
    #     pre_y = reference_odom[np.argmin(descriptor_distances[:,i])][2]

    #     gt_x = reference_odom[sorted_gt_indices[i][0]][1]
    #     gt_y = reference_odom[sorted_gt_indices[i][0]][2]

    #     sns.lineplot(x=np.arange(0,descriptor_distances.shape[0]), y=descriptor_distances[:,i], color='black', linewidth=0.5, ax=ax[1])
    #     if (pre_x - gt_x)*(pre_x - gt_x) + (pre_y - gt_y)*(pre_y - gt_y) <= 25.0:
    #         sns.scatterplot(x=[np.argmin(descriptor_distances[:,i])], y=[np.min(descriptor_distances[:,i])], color='green', s=50, ax=ax[1], label='Correct Prediction')
    #         sns.scatterplot(x=[], y=[], color='red', s=50, ax=ax[1], label='Wrong Prediction')
    #     else:
    #         sns.scatterplot(x=[], y=[], color='green', s=50, ax=ax[1], label='Correct Prediction')
    #         sns.scatterplot(x=[np.argmin(descriptor_distances[:,i])], y=[np.min(descriptor_distances[:,i])], color='red', s=50, ax=ax[1], label='Wrong Prediction')
    #     ax[1].set_title('Distances per query(query index='+str(i)+')')
    #     ax[1].set_xlabel('Reference indices')
    #     ax[1].set_ylabel('Distances')
    #     ax[1].set_ylim(np.min(descriptor_distances), np.max(descriptor_distances))

    #     correct_prediction_handle = plt.scatter([], [], color='green', s=50, label='Correct Prediction')
    #     wrong_prediction_handle = plt.scatter([], [], color='red', s=50, label='Wrong Prediction')
    #     legend_handles = [correct_prediction_handle, wrong_prediction_handle]
    #     legend_labels = [handle.get_label() for handle in legend_handles]
    #     ax[1].legend(handles=legend_handles, labels=legend_labels, loc='upper right')
        
    #     plt.savefig(os.path.join(directory_path, f"{i:03d}" + '.png'))
    #     plt.clf()
    #     plt.close()