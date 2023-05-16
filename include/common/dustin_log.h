#include <glog/logging.h>

#define D_INFO LOG(INFO)
#define D_WARN LOG(WARNING)
#define D_ERROR LOG(ERROR)
#define D_FATAL LOG(FATAL)


#define BASHR "\033[0;31m"
#define BASHG "\033[0;32m"
#define BASHY "\033[0;33m"
#define BASHB "\033[0;34m"
#define BASHW "\033[1;0m"

#define PARAM_LOG(PARAM_NAME) D_INFO << BASHB << "[" << #PARAM_NAME << "] " << BASHW 

#define CLASS_LOG(PARAM_NAME) D_INFO << BASHG << "[" << #PARAM_NAME << "] " << BASHW 