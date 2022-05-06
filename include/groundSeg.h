#ifndef GROUNDSEG_H_
#define GROUNDSEG_H_

#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>


#define delta 0.1f
#define epsilon 0.3f
#define zeta 0.0f
#define f 0.8f


namespace pcl
{
  //template <typename pcl::PointXYZI>
  class GroundSeg: public FilterIndices<pcl::PointXYZI>
  {
    protected:
      using Filter<pcl::PointXYZI>::filter_name_;
      using Filter<pcl::PointXYZI>::getClassName;
      using Filter<pcl::PointXYZI>::input_;
      using Filter<pcl::PointXYZI>::indices_;

      using PointCloud = typename FilterIndices<pcl::PointXYZI>::PointCloud;

    public:
      GroundSeg (const float resolution)
      {
        setResolution (resolution);
        filter_name_ = "GroundSeg";
      }
      ~GroundSeg ()
      {
      }
      inline void setResolution (const float resolution)
      {
        resolution_ = resolution;
        // Use multiplications instead of divisions
        inverse_resolution_ = 1.0f / resolution_;
      }
      inline float getResolution () { return (resolution_); }

    protected:
      /** \brief The resolution. */
      float resolution_;
      /** \brief Internal resolution stored as 1/resolution_ for efficiency reasons. */
      float inverse_resolution_;

      void applyFilter (PointCloud &output) override;

      inline void applyFilter (Indices &indices) override
      {
        applyFilterIndices (indices);
      }

      void applyFilterIndices (Indices &indices);
        
  };
}
#endif