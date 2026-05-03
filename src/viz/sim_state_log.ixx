module;

#include <string_view>

export module mininav.viz.sim_state_log;

import mininav.core.types;
import mininav.viz.rerun_sink;

export namespace mininav
{
    void log_to_rerun(RerunSink& sink,
                      const SimStateV0& state,
                      std::string_view entity_root) noexcept;
}
