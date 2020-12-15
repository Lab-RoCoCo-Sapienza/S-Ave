#include "base_mapper.h"

namespace lucrezio_spme{
  BaseMapper::BaseMapper(){
    _local_map = new SemanticMap();
    _global_map = new SemanticMap();

    _associations.clear();
  }

  BaseMapper::~BaseMapper(){
    delete _local_map;
    delete _global_map;
  }

}

