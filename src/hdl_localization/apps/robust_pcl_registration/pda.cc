#include "robust_pcl_registration/pda.h"

#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

#include "robust_pcl_registration/point_cloud_registration.h"

Ipda::Ipda(const IpdaParameters& params)
  : params_(params) {}

Eigen::Affine3d Ipda::evaluate(
    pcl::PointCloud<PointType>::Ptr source_cloud,
    pcl::PointCloud<PointType>::Ptr target_cloud_origin,
    pcl::PointCloud<PointType>::Ptr aligned_source,
    Eigen::Matrix4d init_transform) {
  CHECK(source_cloud);
  CHECK(target_cloud_origin);

  // preprocess target_cloud
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  double center_x = init_transform.block<3, 1>(0, 3)[0];
  double center_y = init_transform.block<3, 1>(0, 3)[1];
  for (int i = 0; i < (*target_cloud_origin).size(); i++)
  {
    pcl::PointXYZ pt(target_cloud_origin->points[i].x, target_cloud_origin->points[i].y, target_cloud_origin->points[i].z);
    float THRESHOLD = 20.0f;
    if (sqrt((pt.x- center_x) * (pt.x- center_x) + (pt.y- center_y) * (pt.y- center_y)) > THRESHOLD) // e.g. remove all pts below zAvg
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(target_cloud_origin);
  extract.setIndices(inliers);
  extract.setNegative(true);
  pcl::PointCloud<PointType>::Ptr target_cloud = boost::make_shared<pcl::PointCloud<PointType> >();
  extract.filter(*target_cloud);

  Eigen::Affine3d final_transformation, previous_transformation, I_3;
  final_transformation.matrix() = init_transform;
  previous_transformation.setIdentity();
  I_3.setIdentity();

  // pcl::PointCloud<PointType>::Ptr aligned_source =
  //     boost::make_shared<pcl::PointCloud<PointType> >();
  *aligned_source = *source_cloud;
  CHECK(aligned_source);

  pcl::transformPointCloud(*aligned_source, *aligned_source, final_transformation);
  LOG(INFO) << "Init Final Transformation: " << std::endl << final_transformation.matrix();

  point_cloud_registration::PointCloudRegistrationParams params;
  params.dof = params_.dof;
  params.max_neighbours = params_.max_neighbours;
  params.dimension = params_.dimension;

  // Solver parameters.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.use_nonmonotonic_steps = params_.solver_use_nonmonotonic_steps;
  options.minimizer_progress_to_stdout = params_.solver_minimizer_progress_to_stdout;
  options.max_num_iterations = params_.solver_maximum_iterations;
  options.function_tolerance = params_.solver_function_tolerance;
  options.num_threads = params_.solver_num_threads;
  ceres::Solver::Summary summary;

  for (size_t iter = 0u; iter < params_.maximum_iterations; ++iter) {
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(target_cloud);
    std::vector<float> distances;
    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(aligned_source->size(),
                                                               target_cloud->size());
    std::vector<Eigen::Triplet<int> > tripletList;
    for (std::size_t i = 0u; i < aligned_source->size(); i++) {
      std::vector<int> neighbours;
      kdtree.radiusSearch((*aligned_source)[i],
                          params_.radius, neighbours, distances, params_.max_neighbours);
      for (int j : neighbours) {
        tripletList.push_back(Eigen::Triplet<int>(i, j, 1));
      }
    }
    data_association.setFromTriplets(tripletList.begin(), tripletList.end());
    data_association.makeCompressed();

    point_cloud_registration::PointCloudRegistration registration(
          *aligned_source, *target_cloud, data_association, params);
    registration.solve(options, &summary);
    VLOG(100) << summary.FullReport();

    const Eigen::Affine3d current_transformation = registration.transformation();
    final_transformation = current_transformation * final_transformation;
    LOG(INFO) << "Current Transformation: " << std::endl << final_transformation.matrix();
    pcl::transformPointCloud(*aligned_source, *aligned_source, current_transformation);

    // Check convergence.
    const double transformation_epsilon =
        ((current_transformation * previous_transformation.inverse()).matrix()
         - Eigen::Matrix4d::Identity()).norm();
    LOG(INFO) << "Transformation epsilon: " << transformation_epsilon;
    if (transformation_epsilon < params_.transformation_epsilon) {
      LOG(INFO) << "IPDA converged." << std::endl;
      return final_transformation;
    }
    previous_transformation = current_transformation;
  }
}
