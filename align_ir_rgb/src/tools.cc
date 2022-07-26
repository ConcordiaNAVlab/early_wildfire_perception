/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: tools.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-04-18
 *
 *   @Description:
 *
 *******************************************************************************/

#include "align_ir_rgb/tools.h"
#include <boost/filesystem.hpp>

std::vector<std::string> GetSubFolders(const std::string path_name) {
  //<- The path you want to get sub-folders of
  boost::filesystem::path p(path_name);
  boost::filesystem::directory_iterator end_itr;

  // cycle through the directory
  std::vector<std::string> dirs;
  for (boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr) {
    if (is_directory(itr->path())) {
      dirs.push_back(itr->path().string());
    }
  }

  return dirs;
}

/**
 * locate the file to the line number
 * */
std::ifstream& SeekToLine(std::ifstream& in, const uint16_t line_nbr) {
  int i;
  char buf[1024];
  // locate to begin of the file
  in.seekg(0, std::ios::beg);
  for (i = 0; i < line_nbr; i++) {
    in.getline(buf, sizeof(buf));
  }
  return in;
}
