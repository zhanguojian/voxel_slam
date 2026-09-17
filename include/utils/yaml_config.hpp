

#pragma once

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>

class YamlConfig {
 public:

  //显式构造yaml类
  explicit YamlConfig(const std::string &path) {
    try {
      std::ifstream fin(path);
      if (!fin.is_open()) {
        LOG(WARNING) << "配置文件路径错误，请检查：" << path;
        valid_ = false;
        return;
      }

      node_ = YAML::Load(fin);

      valid_ = true;
      LOG(INFO) << "加载配置文件路径为：" << path;

    } catch (const YAML::ParserException &e) 
    {
      LOG(ERROR) << "配置文件格式错误：" << e.what();
      valid_ = false;
    }
  }

  //它就是整个配置类的状态标志。成功为true
  bool valid() const { return valid_; }



  // 读取一级 YAML 配置。
  template <typename T>
  T get(const std::string &key, const T &default_value) const 
  {

    if (!valid_ || !node_[key]) 
    {
      LOG(WARNING) << "使用默认值：" << key << " = " << toString(default_value);
      return default_value;
    }

    try 
    {
      T value = node_[key].as<T>();
      LOG(INFO) << "读取配置项：" << key << " = " << toString(value);
      return value;

    } catch (const YAML::TypedBadConversion<T> &) 
    {
      LOG(WARNING) << "类型转换错误：" << key << "，使用默认值 " << toString(default_value);
      return default_value;
    }
  }


  //多级yaml配置读取
  template <typename T>
  T getPath(const std::string &path, const T &default_value) const 
  {
    if (!valid_) {
      LOG(WARNING) << "非法配置文件，使用默认值：" << path;
      return default_value;
    }

    //读取分级的层
    std::vector<std::string> keys = splitPath(path);

    //获取初始化头节点
    const YAML::Node cur = &node_;

    //逐层读取。  
    for (const auto &key : keys) {
      //node树节点也就是map
      if (!cur->IsMap()) {
        LOG(WARNING) << "配置项不存在：" << path;
        return default_value;
      }

      //更新树的头结点
      const YAML::Node next = (*cur)[key];
      if (!next) {
        LOG(WARNING) << "配置项不存在：" << path;
        return default_value;
      }

      cur = next;  // 继续向下
    }

    try {
      T value = cur->as<T>();
      LOG(INFO) << "配置项：" << path << " = " << toString(value);
      return value;
    } catch (...) {
      LOG(WARNING) << "类型转换错误：" << path << "，使用默认值";
      return default_value;
    }
  }

  void printAllKeys() const 
  {
    if (!valid_) {
      LOG(WARNING) << "配置文件无效，无法打印 keys";
      return;
    }
    printNode(node_, "");
  }

 private:
  YAML::Node node_;     ///< yaml解析节点
  bool valid_ = false;  ///< 是否合法


  static std::vector<std::string> splitPath(const std::string &path) 
  {
    std::vector<std::string> result;
    std::string cur;
    bool escape = false;

    for (char c : path) {
      if (escape) {
        cur.push_back(c);  // 字面字符
        escape = false;
      } else if (c == '\\') {
        escape = true;  // 进入转义状态
      } else if (c == '.') {
        result.push_back(cur);
        cur.clear();
      } else {
        cur.push_back(c);
      }
    }

    result.push_back(cur);
    return result;
  }


  template <typename U>
  static std::string toString(const U &value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
  }


  template <typename U>
  static std::string toString(const std::vector<U> &vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
      oss << vec[i];
      if (i + 1 < vec.size()) {
        oss << ", ";
      }
    }
    oss << "]";
    return oss.str();
  }

  static std::string toString(bool value) { return value ? "true" : "false"; }


  static std::string toString(const std::string &value) { return "\"" + value + "\""; }


  static void printNode(const YAML::Node &node, const std::string &prefix) 
  {
    if (node.IsMap()) {
      for (auto it = node.begin(); it != node.end(); ++it) {
        std::string key = it->first.as<std::string>();
        printNode(it->second, prefix.empty() ? key : prefix + "." + key);
      }
    } else if (node.IsSequence()) {
      for (size_t i = 0; i < node.size(); ++i) {
        printNode(node[i], prefix + "[" + std::to_string(i) + "]");
      }
    } else {
      // Scalar
      LOG(INFO) << prefix << " = " << node.as<std::string>();
    }
  }
};


template <>
inline YAML::Node YamlConfig::get<YAML::Node>(const std::string &key, const YAML::Node &default_value) const 
{
  if (!valid_ || !node_[key]) 
  {
    return default_value;
  }

  return node_[key];
}