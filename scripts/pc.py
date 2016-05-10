#!/usr/bin/env python


import rospy
from std_msgs.msg import Empty
import diagnostic_updater
import diagnostic_msgs
import psutil
import netifaces
from docker import Client
import json

class PC(object):
            

    def docker_diagnostics(self, container):
        c_stats = self.cli.stats(container)
        def update(stats):
            stats.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Running")
            data = json.loads(next(c_stats))
            mem_u = data['memory_stats']['usage']
            mem_t = data['memory_stats']['limit']
            stats.add("memory", "%.3g | %.0g  " % (mem_u/1e6,mem_t/1e6))
            stats.add("memory percent", "%.2g%%" % (100*mem_u/1e6/mem_t/1e6))
            print data
            cpu_usage = data["cpu_stats"]["cpu_usage"]["total_usage"]
            precpu_usage = data["precpu_stats"]["cpu_usage"]["total_usage"]
            system_cpu_usage = data["cpu_stats"]["system_usage"]
            system_precpu_usage = data["precpu_stats"]["system_usage"]
            cpu_delta = cpu_usage - precpu_usage
            system_delta = system_cpu_usage - system_precpu_usage
            if system_delta > 0 and cpu_delta > 0:
                cpu_percent = 100.0 * cpu_delta / system_delta * len(percpu)
                stats.add("cpu" "%.2g%%" % cpu_percent)
            return stats
        return update

    def cpu_diagnostics(self, stat):
        cpu = psutil.cpu_percent()
        if cpu > 80:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Using almost all cpu")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
        stat.add("cpu percent", cpu)
        return stat    


    def memory_diagnostics(self, stat):
        mem = psutil.phymem_usage()
        if mem.percent >80 :
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Using almost all memory")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
        stat.add("memory percent", mem.percent)
        stat.add("memory usage (MB)", int(mem.used/1e6))
        return stat    


    def network_diagnostics(self, iface):
        def update(stat):
            if iface not in netifaces.interfaces():
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "%s is not defined" % iface)
                return stat
            ip =netifaces.ifaddresses(iface)[2]
            if not ip or not ip[0]['addr']:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "%s is down, no ip assigned" % iface)
                return stat
            net = psutil.network_io_counters(pernic=True).get(iface)
            if net.errin or net.errout:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Some error packets")
            elif net.dropin or net.dropout:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Some packets have been dropped")
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
            for k,v in net._asdict().items():
                 stat.add(k, v)
        return update


    def __init__(self):

        rospy.init_node('pc_controller', anonymous=False)

        self.cli = Client(base_url='unix://var/run/docker.sock', version='1.22')
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("macmini")
        self.updater.add("CPU", self.cpu_diagnostics)
        self.updater.add("Memory", self.memory_diagnostics)
        for iface in ['wlan0','eth0']:
            self.updater.add("Network %s" % iface, self.network_diagnostics(iface))

        for container in ['romantic_hypatia','high_engelbart']:
            self.updater.add("Docker %s" % container, self.docker_diagnostics(container))

        rospy.Subscriber("shutdown", Empty, self.has_received_shutdown)

        while not rospy.is_shutdown():
            rospy.sleep(1)
            self.updater.update()
        
    def has_received_shutdown(self,msg):
        rospy.loginfo("Begin shutdown");

if __name__ == '__main__':
    try:
        PC()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller finished.")
