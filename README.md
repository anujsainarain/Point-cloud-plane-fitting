# Point-cloud-plane-fitting
This repository contains a custom implementation of the Random Sample Consensus (RANSAC) algorithm for fitting a plane on 3D point clouds. The provided code snippet utilizes Open3D to load and visualize a demo point cloud, showcasing the application of the implemented algorithm.

<div style="text-align:center">
  <img src="https://github.com/Amenephous/Point-cloud-plane-fitting/assets/48127920/04fa0bdd-2e72-499a-80ee-aae9bd2ed233" alt="Original Data">
  <p>The original 3D point cloud data on which a plane is to be fitted. Outliers are present, and the goal is to identify the best-fit plane using RANSAC.</p>
</div>

<div style="text-align:center">
  <img src="https://github.com/Amenephous/Point-cloud-plane-fitting/assets/48127920/5d701b8b-efb7-4d0f-b7ec-3ec071f40c07" width="500" height="350" alt="Plane Fitted with RANSAC">
  <p>The result after fitting a plane using the RANSAC algorithm. Outliers have been eliminated, and the best-fit plane is highlighted in the point cloud data.</p>
</div>
