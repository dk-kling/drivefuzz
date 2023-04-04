# ==============================================================================
# -- Parse arguments -----------------------------------------------------------
# ==============================================================================

DOC_STRING="Retrieve the plugins for CARLA"

USAGE_STRING="Usage: $0 [-h|--help] [--release]"

RELEASE=false

if [ $? != 0 ] ; then echo "$USAGE_STRING" ; exit 2 ; fi

while true; do
  case "$1" in
    --release )
      RELEASE=true
      shift ;;
    -h | --help )
      echo "$DOC_STRING"
      echo "$USAGE_STRING"
      exit 1
      ;;
    * )
      break ;;
  esac
done

# ==============================================================================
# -- Get Plugins ---------------------------------------------------------------
# ==============================================================================

log "Retrieving Plugins"

if [[ -d "${CARLA_ROOT_FOLDER}Plugins" ]] ; then
  log "Plugins already installed."
else
  if ${RELEASE} ; then
    git clone --depth=1 --recursive https://github.com/carla-simulator/carla-plugins.git "${CARLA_ROOT_FOLDER}Plugins"
  else
    git clone --recursive https://github.com/carla-simulator/carla-plugins.git "${CARLA_ROOT_FOLDER}Plugins"
  fi
fi
