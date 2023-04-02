#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "image_utils.h"

TEST(test_plan_utils, priority_queue_with_compare) {
    Node n1{0, 0, 37};
    Node n2{0, 0, 33};
    Node n3{0, 0, 29};
    priority_queue<Node, vector<Node>, Compare> pq;
    pq.push(n1);
    pq.push(n2);
    pq.push(n3);
    int cost_vec[3] = {29, 33, 37};
    int i = 0;
    while (!pq.empty()) {
        ASSERT_EQ(pq.top().cost, cost_vec[i]);
        pq.pop(); // heapify happens
        i += 1;
    }
}

// TEST(test_plan_utils, cost_grid_generation){
//   cv::Mat opencv_img = load_image("/Users/thayjessrivas/Documents/scythepathplanningchallenge/path_planning_challenge_image_3.png");
//   floatGrid grid = img_to_vector(opencv_img);
//   int rows = grid.size();
//   int cols = grid[0].size();
//   intGrid visited;
//   floatGrid cost_grid;
//   PixelWithCost goal{50, 50, 0.0};
//   cost_grid = getCostGrid(grid, goal);
//   ASSERT_EQ(cost_grid[50][50], grid[50][50]);
//   ASSERT_EQ(cost_grid[0][0], grid[0][0] + 100.0);
//   // ASSERT_EQ(visited[rows-1][cols-1], 0);
// }

// TEST(test_plan_utils, valid_neighbors){
//   cv::Mat opencv_img = load_image("/Users/thayjessrivas/Documents/scythepathplanningchallenge/path_planning_challenge_image_3.png");
//   floatGrid grid = img_to_vector(opencv_img);
//   floatGrid cost_grid;
//   PixelWithCost goal{50, 50, 0.0};
//   cost_grid = getCostGrid(grid, goal);
//   grid[20][20] = 255.0;
//   grid[19][19] = 255.0;
//   grid[19][20] = 255.0;
//   grid[20][19] = 255.0;
//   intGrid visited;
//   visited.resize(grid.size(), std::vector<int>(grid[0].size()));
//   for(int i = 0; i < visited.size(); ++i){
//     for(int j = 0; j < visited[i].size(); ++j){
//       visited[i][j] = 0;
//     }
//   }
//   std::vector<PixelWithCost> neighbors = validNeighbors({18, 19, 0}, visited, cost_grid, grid);
//   ASSERT_EQ(neighbors.size(), 6);
//   neighbors = validNeighbors({0, 0, 0}, visited, cost_grid, grid);
//   ASSERT_EQ(neighbors.size(), 3);
// }

TEST(test_plan_utils, a_star){
  std::string img_path = "/Users/thayjessrivas/Documents/scythepathplanningchallenge/path_planning_challenge_image_1.png";
  // cv::Mat opencv_img = load_image("/Users/thayjessrivas/Documents/scythepathplanningchallenge/path_planning_challenge_image_3.png");
  cv::Mat opencv_img = cv::imread(img_path);
  cv::Mat img_bgr = cv::imread(img_path);


  floatGrid grid = img_to_vector(opencv_img);
  // for(int r = 0; r < img_bgr.rows; ++r) {
  //   for(int c = 0; c < img_bgr.cols; ++c) {
  //     if(img_bgr.at<cv::Vec3b>(r,c)[0] == 255){
  //       std::cout << "Pixel (row, col) =  : (" << r << ", " << c << ") =" <<
  //            img_bgr.at<cv::Vec3b>(r,c) << std::endl;
  //       std::cout << std::format("Grid at same position = {}", grid[r][c]) << std::endl;
  //     }
  //   }
  // }
  floatGrid cost_grid;
  PixelWithCost goal{900, 800, 0.0};
  cost_grid = getCostGrid(grid, goal);
  intGrid visited;
  visited.resize(grid.size(), std::vector<int>(grid[0].size()));
  for(int i = 0; i < visited.size(); ++i){
    for(int j = 0; j < visited[i].size(); ++j){
      visited[i][j] = 0;
    }
  }
  // cv::Mat dst;
  // opencv_img.convertTo(dst, CV_32F);
  std::vector<PixelWithCost> path;
  bool success;
  // grid[10][10] = 255.0;
  // grid[50][50] = 255.0; 
  success = aStarSearch(path, {0, 0, 0.0}, goal, cost_grid, grid);
  // for(auto p : path){
    // p.print();
    // std::cout << std::format("Grid value = {}", grid[p.row][p.col]) << std::endl;
    // std::cout << "Image value = " << opencv_img.at<cv::Vec3b>(p.row, p.col) << std::endl;
  // }
  // new_img = vector_to_img(grid);
  draw_path_on_img(path, img_path);
}

// TEST(test_image_utils, grid_from_image) {
//   cv::Mat opencv_img = load_image("/Users/thayjessrivas/Documents/scythepathplanningchallenge/path_planning_challenge_image_3.png");
//   floatGrid grid = img_to_vector(opencv_img);
//   // Need a test on the image loaded itself to check if all the pixel values are 0 and 255
//   std::cerr << "img size = " << opencv_img.size() << std::endl;
//   for(int row = 0; row < grid.size(); ++row){
//       ASSERT_THAT(grid[row], testing::Each(testing::AnyOf(testing::Eq(0.0), testing::Eq(255.0))));
//   }
// }

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
