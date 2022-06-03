#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/extract_indices.h>
#include "../include/groundSeg.h" //VSCODE nao reconhece "groundSeg.h" - só para tirar squiggle do VSCODE

#include <chrono>
using namespace std::chrono;

struct point_index_id
{
  int id;
  unsigned int cloud_point_index;

  point_index_id (int id_, unsigned int cloud_point_index_) : id (id_), cloud_point_index (cloud_point_index_) {}
  bool operator < (const point_index_id &p) const { return (id < p.id); }
};

int HashThem(int a, int b)
{
    uint A = (a >= 0 ? 2 * a : -2 * a - 1);
    uint B = (b >= 0 ? 2 * b : -2 * b - 1);
    int C = ((A >= B ? A * A + A + B : A + B * B) / 2);
    return a < 0 && b < 0 || a >= 0 && b >= 0 ? C : -C - 1;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pcl::GroundSeg::getGround(PointCloud &ground)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  inliers->indices = nonGroundIndices;

  extract.setInputCloud (input_);
  extract.setIndices(inliers);
  extract.setNegative (true);
  extract.filter (ground);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pcl::GroundSeg::applyFilter (PointCloud &output)
{
  //Indices indices;
  output.is_dense = true;
  applyFilterIndices (nonGroundIndices);
  pcl::copyPointCloud<pcl::PointXYZ> (*input_, nonGroundIndices, output);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pcl::GroundSeg::applyFilterIndices (Indices &indices)
{
  auto start_division = high_resolution_clock::now();

  std::vector<point_index_id> index_vector;
  index_vector.reserve (indices_->size ());
  
  for (const auto& index : (*indices_))
  {   
    int idx = std::floor ((*input_)[index].x / resolution_);
    int idy = std::floor ((*input_)[index].y / resolution_);

    // Compute the grid cell index
    index_vector.emplace_back(HashThem(idx, idy), index);
  }

  //sort from smaller id to bigger id (points with same id will be next to each other)
  std::sort (index_vector.begin (), index_vector.end (), std::less<point_index_id> ());

  // discover index of first point with x id and last 
  unsigned int index = 0;
  std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;

  first_and_last_indices_vector.reserve (index_vector.size ());
  while (index < index_vector.size ())
  {
    unsigned int i = index + 1;
    while (i < index_vector.size () && index_vector[i].id == index_vector[index].id)
      ++i;
    first_and_last_indices_vector.emplace_back(index, i);
    index = i;
  }


  auto stop_division = high_resolution_clock::now();
  auto start_classification = high_resolution_clock::now();


  //for each grid cell
  for (const auto &cp : first_and_last_indices_vector)
  {
    unsigned int first_index = cp.first;
    unsigned int last_index = cp.second;

    float min_z = (*input_)[index_vector[first_index].cloud_point_index].z;
    float max_z = (*input_)[index_vector[first_index].cloud_point_index].z;

    //Discover maximum and minimum Z
    for (unsigned int i = first_index + 1 ; i < last_index; ++i)
    {
      if ((*input_)[index_vector[i].cloud_point_index].z < min_z)
      {
        min_z = (*input_)[index_vector[i].cloud_point_index].z;
      }
      if ((*input_)[index_vector[i].cloud_point_index].z > max_z)
      {
        max_z = (*input_)[index_vector[i].cloud_point_index].z;
      }
    }
   
    //Classification
    if(min_z < zeta)
    {
      if((max_z - min_z) > epsilon) // if the object is taller than epsilon (Large Object)
      {
        for (unsigned int i = first_index; i < last_index; ++i) //for each point of the cell
        {
          if((((*input_)[index_vector[i].cloud_point_index].z) >= (min_z + delta)))
          {
            indices.push_back(index_vector[i].cloud_point_index); //Push to Non Ground vector
          }  
        }
      }
      else if(((max_z - min_z) < epsilon) && ((max_z - min_z) > gamma)) // if the object is smaller than epsilon and taller than gamma (Small Object)
      {
        for (unsigned int i = first_index; i < last_index; ++i) //for each point of the cell
        {
          if((((*input_)[index_vector[i].cloud_point_index].z) >= (min_z + ((max_z - min_z) / f))))
          {
            indices.push_back(index_vector[i].cloud_point_index); //Push to Non Ground vector
          } 
        }
      }
    }
    else //all the points of the cell are above sensor height
    {
      for (unsigned int i = first_index; i < last_index; ++i)//for each point of the cell
      {
        indices.push_back(index_vector[i].cloud_point_index); // Push to Non Ground vector
      }
    }
  }
  
  auto stop_classification = high_resolution_clock::now();

  auto division_duration = duration_cast<microseconds>(stop_division - start_division);
  auto classification_duration = duration_cast<microseconds>(stop_classification - start_classification);
  PCL_WARN("Grid Divison Time(secs): %f\n",division_duration.count()/1000000.0f);
  PCL_WARN("Classification Time(secs): %f\n",classification_duration.count()/1000000.0f);
  PCL_WARN("Total Time(secs): %f\n",((division_duration.count() + classification_duration.count())/1000000.0f));
}