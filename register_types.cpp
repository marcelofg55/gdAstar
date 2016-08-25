#include "register_types.h"
#include "object_type_db.h"
#include "gdastar.h"

void register_gdAstar_types() {
	ObjectTypeDB::register_type<gdAstar>();
}

void unregister_gdAstar_types() {
	//nothing to do here
}
