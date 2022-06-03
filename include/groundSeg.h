#ifndef GROUNDSEG_H_
#define GROUNDSEG_H_

#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>


#define delta 0.1f
#define epsilon 0.3f
#define zeta 0.0f
#define gamma 0.15f
#define f 5.0f

namespace pcl
{
  class GroundSeg: public FilterIndices<pcl::PointXYZ>
  {
    protected:
      using Filter<pcl::PointXYZ>::filter_name_;
      using Filter<pcl::PointXYZ>::getClassName;
      using Filter<pcl::PointXYZ>::input_;
      using Filter<pcl::PointXYZ>::indices_;

      using PointCloud = typename FilterIndices<pcl::PointXYZ>::PointCloud;

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
      Indices nonGroundIndices;

    protected:
      /** \brief The resolution. */
      float resolution_;

      void applyFilter (PointCloud &output) override;

      inline void applyFilter (Indices &indices) override
      {
        applyFilterIndices (indices);
      }

      void applyFilterIndices (Indices &indices);
  };
}
#endif