// Copyright 2023 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COMMON__CONFIG__ERROR_HPP_
#define COMMON__CONFIG__ERROR_HPP_

#include <stdexcept>
#include <string>

namespace diagnostic_graph_aggregator
{

struct TreePath
{
  explicit TreePath(const std::string & file);
  TreePath field(const std::string & name);
  TreePath child(const std::string & path);
  TreePath child(const std::string & path, const size_t index);
  TreePath child(const size_t index);
  std::string text() const;
  std::string file() const { return file_; }
  std::string node() const { return node_; }
  std::string name() const { return name_; }

private:
  std::string file_;
  std::string node_;
  std::string name_;
};

struct Exception : public std::runtime_error
{
  using runtime_error::runtime_error;
};

struct FieldNotFound : public Exception
{
  explicit FieldNotFound(const TreePath & path) : Exception(format(path)) {}
  static std::string format(const TreePath & path)
  {
    return "required field is not found: " + path.name() + path.text();
  }
};

struct InvalidType : public Exception
{
  explicit InvalidType(const TreePath & path) : Exception(format(path)) {}
  static std::string format(const TreePath & path)
  {
    return "field is not a list type: " + path.name() + path.text();
  }
};

struct UnknownSubstitution : public Exception
{
  explicit UnknownSubstitution(const std::string & substitution, const TreePath & path)
  : Exception(format(substitution, path))
  {
  }
  static std::string format(const std::string & substitution, const TreePath & path)
  {
    return "unknown substitution: " + substitution + path.text();
  }
};

/*
struct FileNotFound : public Exception
{
  using Exception::Exception;
};

struct UnknownType : public Exception
{
  using Exception::Exception;
};

struct InvalidValue : public Exception
{
  using Exception::Exception;
};

struct PathConflict : public Exception
{
  using Exception::Exception;
};

struct PathNotFound : public Exception
{
  using Exception::Exception;
};

struct GraphStructure : public Exception
{
  using Exception::Exception;
};
*/

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__ERROR_HPP_
