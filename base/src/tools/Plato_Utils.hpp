namespace Plato
{
namespace Utils
{
template<typename T>
T* byName(const std::vector<T*> & aArgumentVector, const std::string & aName);
}
}

template <typename T>
T* Plato::Utils::byName(const std::vector<T*> & aArgumentVector, const std::string & aName)
{
  for( T* item : aArgumentVector ){
    if( item->myName() == aName ){
      return item;
    }
  }
  return nullptr;
}

