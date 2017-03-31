#!/usr/bin/env python

import rospy

from pitasc_library.scripts.service_caller  import ServiceCallerNEW, EmptyService

class StartService(ServiceCallerNEW):
    """Call a srvs of type 'snapshot_tools/String'."""

    def __init__(self, service_name, video_name, namespace=None,
                 on_start=True, wait_for_service=True):
        from snapshot_tools.srv import String, StringRequest

        req = StringRequest()
        req.str = video_name

        ServiceCallerNEW.__init__(
            self, service_name, namespace,
            String, req, on_start, wait_for_service)

class StopService(EmptyService):
    pass

# eof
