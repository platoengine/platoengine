#ifndef PLATO_INPUT_DATA_H
#define PLATO_INPUT_DATA_H

#include <map>
#include <memory>
#include <vector>

#ifdef USE_CXX_14
#include <experimental/any>
namespace Plato {
  using any = std::experimental::any;
  template <typename T>
  T any_cast(any anyVal){return std::experimental::any_cast<T>(anyVal);}
}
#else
#include <Teuchos_any.hpp>
#include <iostream>
namespace Plato {
  using any = Teuchos::any;
  template <typename T>
  T any_cast(any anyVal){return Teuchos::any_cast<T>(anyVal);}
}
#endif

namespace Plato {

class InputData {
 public:
  InputData();
  InputData(std::string name);

  std::string name() const;
  bool empty() const;

  /*! Set a {name,value} pair in the InputData.
   *  If an entry by this name already exists and the typeids match, the value in the InputData
   *    is changed.  
   *  If an entry by this name already exists and the typeids do not match, an exception is thrown.
   */
  template <typename T>
  void set(std::string name, const T& value, unsigned int index = 0)
  {
    auto num = m_parameters->count(name);

    if( index == num ){
      Plato::any newEntry = Plato::any(value);
      m_parameters->insert(std::pair<std::string, Plato::any>(name, newEntry));
    } else 
    if( index > num ){
      // attempted to set an entry that doesn't exist
      throw 1;
    } else {
      auto it = m_parameters->lower_bound(name);
      it = std::next(it, index);
      auto& anyVal = it->second;
      if( typeid(T) == anyVal.type() ){
        anyVal = Plato::any(value);
      } else {
        // entry exists, but is not of type T.
        throw 1;
      }
    }
  }

  /*! Get the number of entries with this name and type.  Currently, the add function enforces that all
   *  entries that have the same name are the same type.  This function will still work if that requirement
   *  is eliminated.
   */ 
 
  template <typename T>
  unsigned int size(std::string name) const
  {
    unsigned int numWithNameAndType = 0;
    for( auto it  = m_parameters->lower_bound(name); 
              it != m_parameters->upper_bound(name); ++it ){
      auto& anyVal = it->second;
      if( typeid(T) == anyVal.type() ){
        numWithNameAndType++;
      }
    }
    return numWithNameAndType;
  }

  /*! Get the unique value that maps to the given name.
   *  If an entry by this name doesn't exist an exception is thrown.
   *  If an entry by this name exists but the typeid doesn't match the template parameter type, an exception is thrown.
   */ 
 
  template <typename T>
  T get(std::string name, unsigned int index = 0) const
  {
    auto num = m_parameters->count(name);

    if( index >= num ){
      // requested an entry that doesn't exist.
      throw 1;
    } else {
      auto it = m_parameters->lower_bound(name);
      it = std::next(it, index);
      auto& anyVal = it->second;
      if( typeid(T) == anyVal.type() ){
        return Plato::any_cast<T>(anyVal);
      } else {
        // entry exists, but is not of type T.
        throw 1;
      }
    }
  }

  /*! Add a {name, value} pair in the InputData.  The name is not required to be unique, however, 
   *  if entries by this name already exist the value to be added must match their type.
   *   
   */
  template <typename T>
  void add(std::string name, const T& value)
  {
    auto range = m_parameters->equal_range(name);
    for(auto it=range.first; it!=range.second; ++it){
      auto& anyVal = it->second;
      if( typeid(T) != anyVal.type() ){
        // non-unique entries are required to be of the same type.
        throw 1;
      }
    }
    m_parameters->insert(std::pair<std::string, Plato::any>(name, Plato::any(value)));
  }


  /*! Get the range of entries with this name.
   *
   */
  template <typename T>
  std::vector<T> getByName(std::string name) const
  {
    std::vector<T> retVector;
    auto range = m_parameters->equal_range(name);
    for( auto it = range.first; it != range.second; ++it){
      auto& anyVal = it->second;
      if( typeid(T) == anyVal.type() ){
        retVector.push_back(Plato::any_cast<T>(anyVal));
      }
    }
    return retVector;
  }
  
#ifndef USE_CXX_14
  friend bool operator==(const InputData& l, const InputData& r);
  friend std::ostream& operator<<(std::ostream& os, const InputData& v);
#endif


 private:
  std::string m_name;
  std::shared_ptr<std::multimap<std::string, Plato::any>> m_parameters;

  void init();
};

} // end namespace Plato
#endif

