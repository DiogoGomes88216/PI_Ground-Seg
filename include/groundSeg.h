#ifndef GROUNDSEG_H_
#define GROUNDSEG_H_

#include <pcl/filters/filter_indices.h>

#define delta 0.1f
#define epsilon 0.3f
#define zeta 0.0f
#define gamma 0.15f
#define f 5.0f

namespace pcl
{
  class GroundSeg: public FilterIndices<PointXYZ>
  {
    protected:
      using Filter<PointXYZ>::filter_name_;
      using Filter<PointXYZ>::getClassName;
      using Filter<PointXYZ>::input_;
      using Filter<PointXYZ>::indices_;

      using PointCloud = typename FilterIndices<PointXYZ>::PointCloud;

    public:
      GroundSeg (const float resolution)
      {
        setResolution (resolution);
        filter_name_ = "GroundSeg";
      }
      ~GroundSeg (){}
      
      inline void setResolution (const float resolution)
      {
        resolution_ = resolution;
      }
      inline float getResolution () { return (resolution_); }

      void getGround(PointCloud &ground);
      std::vector<int> nonGroundIndices;

    protected:
      /** \brief The resolution. */
      float resolution_;

      void applyFilter (PointCloud &output) override;

      inline void applyFilter (std::vector<int> &indices) override
      {
        applyFilterIndices (indices);
      }

      void applyFilterIndices (std::vector<int> &indices);
  };
}
#endif