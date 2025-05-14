#include "people_tracking_filter/mcpdf_vector.h"
#include "people_tracking_filter/state_pos_vel.h"
#include <assert.h>
#include <vector>
#include <std_msgs/msg/float64.hpp>
#include "people_tracking_filter/rgb.h"


using namespace MatrixWrapper;
using namespace BFL;

static const unsigned int NUM_CONDARG   = 1;


MCPdfVector::MCPdfVector(unsigned int num_samples)
  : MCPdf<tf2::Vector3> (num_samples, NUM_CONDARG)
{}

MCPdfVector::~MCPdfVector() {}


WeightedSample<tf2::Vector3>
MCPdfVector::SampleGet(unsigned int particle) const
{
  assert((int)particle >= 0 && particle < _listOfSamples.size());
  return _listOfSamples[particle];
}


tf2::Vector3 MCPdfVector::ExpectedValueGet() const
{
  tf2::Vector3 pos(0, 0, 0);
  double current_weight;
  std::vector<WeightedSample<tf2::Vector3> >::const_iterator it_los;
  for (it_los = _listOfSamples.begin() ; it_los != _listOfSamples.end() ; it_los++)
  {
    current_weight = it_los->WeightGet();
    pos += (it_los->ValueGet() * current_weight);
  }

  return tf2::Vector3(pos);
}


/// Get evenly distributed particle cloud
void MCPdfVector::getParticleCloud(const tf2::Vector3& step, double threshold, sensor_msgs::msg::PointCloud& cloud) const
{
  unsigned int num_samples = _listOfSamples.size();
  assert(num_samples > 0);
  tf2::Vector3 m = _listOfSamples[0].ValueGet();
  tf2::Vector3 M = _listOfSamples[0].ValueGet();

  // calculate min and max
  for (unsigned int s = 0; s < num_samples; s++)
  {
    tf2::Vector3 v = _listOfSamples[s].ValueGet();
    for (unsigned int i = 0; i < 3; i++)
    {
      if (v[i] < m[i]) m[i] = v[i];
      if (v[i] > M[i]) M[i] = v[i];
    }
  }

  // get point cloud from histogram
  Matrix hist = getHistogram(m, M, step);
  unsigned int row = hist.rows();
  unsigned int col = hist.columns();
  unsigned int total = 0;
  unsigned int t = 0;
  for (unsigned int r = 1; r <= row; r++)
    for (unsigned int c = 1; c <= col; c++)
      if (hist(r, c) > threshold) total++;
  cout << "size total " << total << endl;

  vector<geometry_msgs::msg::Point32> points(total);
  vector<float> weights(total);
  sensor_msgs::msg::ChannelFloat32 channel;
  for (unsigned int r = 1; r <= row; r++)
    for (unsigned int c = 1; c <= col; c++)
      if (hist(r, c) > threshold)
      {
        for (unsigned int i = 0; i < 3; i++)
          points[t].x = m[0] + (step[0] * r);
        points[t].y = m[1] + (step[1] * c);
        points[t].z = m[2];
        weights[t] = rgb[999 - (int)trunc(max(0.0, min(999.0, hist(r, c) * 2 * total * total)))];
        t++;
      }
  cout << "points size " << points.size() << endl;
  cloud.header.frame_id = "base_link";
  cloud.points  = points;
  channel.name = "rgb";
  channel.values = weights;
  cloud.channels.push_back(channel);
}


/// Get histogram from pos
MatrixWrapper::Matrix MCPdfVector::getHistogram(const tf2::Vector3& m, const tf2::Vector3& M, const tf2::Vector3& step) const
{
  unsigned int num_samples = _listOfSamples.size();
  unsigned int rows = round((M[0] - m[0]) / step[0]);
  unsigned int cols = round((M[1] - m[1]) / step[1]);
  Matrix hist(rows, cols);
  hist = 0;

  // calculate histogram
  for (unsigned int i = 0; i < num_samples; i++)
  {
    tf2::Vector3 rel(_listOfSamples[i].ValueGet() - m);
    unsigned int r = round(rel[0] / step[0]);
    unsigned int c = round(rel[1] / step[1]);
    if (r >= 1 && c >= 1 && r <= rows && c <= cols)
      hist(r, c) += _listOfSamples[i].WeightGet();
  }

  return hist;
}



unsigned int
MCPdfVector::numParticlesGet() const
{
  return _listOfSamples.size();
}


