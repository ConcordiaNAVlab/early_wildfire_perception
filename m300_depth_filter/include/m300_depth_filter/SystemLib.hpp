/*******************************************************************************
 *
 *   Copyright (C) 2021 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: SystemLib.hpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2021-10-25
 *
 *   @Description:
 *
 ******************************************************************************/

#ifndef INCLUDE_TOOLS_SYSTEMLIB_HPP_
#define INCLUDE_TOOLS_SYSTEMLIB_HPP_

#include "./PrintCtrlMacro.h"
#include <sys/time.h>
#include <yaml-cpp/yaml.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

namespace depth_filter {

/* return as ms */
inline int32_t getSysTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

/* return as ms */
inline int32_t getTimeInterval(const int32_t begin_time) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000 + tv.tv_usec / 1000 - begin_time);
}

template <typename T>
T getParam(const YAML::Node& node, const std::string& paramName,
           const T& defaultValue) {
  T v;
  try {
    v = node[paramName].as<T>();
    std::cout << "Found parameter: " << paramName << ", value: " << v
              << std::endl;
  } catch (std::exception e) {
    v = defaultValue;
    std::cout << "Cannot find value for parameter: " << paramName
              << ", assigning default: " << v << std::endl;
  }
  return v;
}

/* create a dir (must be single level) if not exists */
inline bool creatDir(const std::string dir) {
  if (access(dir.c_str(), 0) == -1) {
    int flag = mkdir(dir.c_str(), S_IRWXU);
    if (flag == 0) {
      PRINT_INFO("dir: %s created successfully!", dir.c_str());
    } else {
      PRINT_ERROR("dir: %s created failed!", dir.c_str());
      return false;
    }
  } else {
    PRINT_WARN("dir: %s already exists!", dir.c_str());
  }
  return true;
}

/* remove single file */
inline bool removeFile(const std::string filename) {
  if (access(filename.c_str(), 0) == -1) {
    PRINT_WARN("filename: %s does not exit!", filename.c_str());
    return false;
  }

  if (0 == remove(filename.c_str())) {
    PRINT_INFO("remove file %s successfully!", filename.c_str());
    return true;
  } else {
    PRINT_ERROR("remove file %s failed!", filename.c_str());
    return false;
  }
}

// https://blog.csdn.net/cheyo/article/details/6595955
inline bool shellStatus(pid_t& status) {
  if ((-1 != status) && (WIFEXITED(status)) && (0 == WEXITSTATUS(status))) {
    return true;
  }

  return false;
}

inline bool shellRm(const std::string path) {
  PRINT_WARN("rm -rf %s", path.c_str());
  pid_t status = std::system(("rm -rf " + path).c_str());
  return shellStatus(status);
}

inline bool shellMkdir(const std::string path) {
  PRINT_WARN("mkdir -p %s", path.c_str());
  pid_t status = std::system(("mkdir -p " + path).c_str());
  return shellStatus(status);
}

/**
 * locate the file to the line number
 * */
inline std::ifstream& SeekToLine(std::ifstream& in, const uint16_t line_nbr) {
  int i;
  char buf[1024];
  // locate to begin of the file
  in.seekg(0, std::ios::beg);
  for (i = 0; i < line_nbr; i++) {
    in.getline(buf, sizeof(buf));
  }
  return in;
}

inline size_t GetFileNum(const std::string& path) {
  size_t fileNum = 0;
  DIR* pDir;
  struct dirent* ptr;

  if (!(pDir = opendir(path.c_str()))) return fileNum;

  while ((ptr = readdir(pDir)) != 0) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
      fileNum++;
  }

  closedir(pDir);
  return fileNum;
}

}  // namespace depth_filter

#endif  // INCLUDE_TOOLS_SYSTEMLIB_HPP_
