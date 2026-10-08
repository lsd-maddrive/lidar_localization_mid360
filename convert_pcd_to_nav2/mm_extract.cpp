#include <mrpt/io/CFileInputStream.h>
#include <mrpt/img/TColor.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>

int main(int argc, char** argv)
{
  if (argc != 3) return 2;
  mrpt::io::CFileInputStream file(argv[1]);
  // Skip metric_map_t and the KeyframePointCloudMap object header/version.
  std::array<char, 107> header{};
  if (file.Read(header.data(), header.size()) != header.size()) return 3;
  auto ar = mrpt::serialization::archiveFrom(file);

  const auto creation_version = ar.ReadAs<uint8_t>();
  uint32_t u32; uint64_t u64; double d; bool b; uint8_t u8;
  ar >> u32 >> u32;
  if (creation_version >= 1) ar >> d >> u32;
  if (creation_version >= 2) ar >> b >> d;
  if (creation_version >= 3) ar >> u32 >> d;
  if (creation_version >= 4) ar >> b;
  if (creation_version >= 5) ar >> b;
  if (creation_version >= 6) ar >> b;
  if (creation_version >= 7) ar >> u32 >> d;
  if (creation_version >= 8) ar >> d;
  if (creation_version >= 9) ar >> d;
  if (creation_version >= 10) u8 = ar.ReadAs<uint8_t>();

  int8_t insertion_version; ar >> insertion_version >> d;
  u8 = ar.ReadAs<uint8_t>();
  int8_t render_version; ar >> render_version;
  float f; mrpt::img::TColorf color; int8_t colormap; std::string s;
  ar >> f >> color >> colormap;
  if (render_version >= 2) ar >> s; else u8 = ar.ReadAs<uint8_t>();
  if (render_version >= 1) ar >> u64 >> u64;
  if (render_version >= 3) ar >> f;
  if (render_version >= 4) ar >> b >> u32;

  const auto keyframes = ar.ReadAs<uint32_t>();
  std::ofstream out(argv[2]);
  if (!out) return 4;
  size_t points = 0;
  for (uint32_t i = 0; i < keyframes; ++i)
  {
    uint64_t id; mrpt::Clock::time_point timestamp; mrpt::poses::CPose3D pose;
    ar >> id >> timestamp >> pose;
    if (ar.ReadAs<uint8_t>() == 0) continue;
    auto obj = ar.ReadObject();
    auto cloud = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(obj);
    if (!cloud) return 5;
    const auto& xs = cloud->getPointsBufferRef_x();
    const auto& ys = cloud->getPointsBufferRef_y();
    const auto& zs = cloud->getPointsBufferRef_z();
    for (size_t j = 0; j < cloud->size(); ++j)
    {
      mrpt::math::TPoint3D p;
      pose.composePoint(mrpt::math::TPoint3D{xs[j], ys[j], zs[j]}, p);
      out << p.x << ' ' << p.y << ' ' << p.z << '\n';
      ++points;
    }
    if (ar.ReadAs<uint8_t>()) { std::string blob; ar >> blob; }
    if (ar.ReadAs<uint8_t>())
    {
      const auto n = ar.ReadAs<uint32_t>();
      for (uint32_t j = 0; j < n; ++j) { mrpt::math::CMatrixFloat33 cov; ar >> cov; }
    }
  }
  std::cerr << "keyframes=" << keyframes << " points=" << points << '\n';
}
