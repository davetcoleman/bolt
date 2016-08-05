/* From http://stackoverflow.com/questions/669438/how-to-get-memory-usage-at-run-time-in-c
 * 2/23/2016
 * Retrieved by Dave Coleman
 */

#ifndef CURIE_DEMOS_PROCESS_MEM_USAGE_H
#define CURIE_DEMOS_PROCESS_MEM_USAGE_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <unistd.h>

/**
 * \brief processMemUsage(double &, double &) - takes two doubles by reference,
 *        attempts to read the system-dependent data for a process' virtual memory
 *        size and resident set size, and
 * NOTE:
 *        Resident Set Size - this is the combined 'Shared Memory' and 'Memory' in the system monitor
 *        Virtual Memory - swap space on hard disk, I think
 * \return the results in MB. On failure, returns 0.0, 0.0
 */
void processMemUsage(double& vm_usage, double& resident_set)
{
  using std::ios_base;
  using std::ifstream;

  vm_usage = 0.0;
  resident_set = 0.0;

  // 'file' stat seems to give the most reliable results
  //
  std::ifstream stat_stream("/proc/self/stat", ios_base::in);

  // dummy vars for leading entries in stat that we don't care about
  //
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string O, itrealvalue, starttime;

  // the two fields we want
  //
  unsigned long vsize;
  long rss;

  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt >>
      majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >> nice >> O >> itrealvalue >> starttime >>
      vsize >> rss;  // don't care about the rest

  stat_stream.close();

  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;  // in case x86-64 is configured to use 2MB pages
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;

  // Convert to MB
  vm_usage = vm_usage / 1024.0;
  resident_set = resident_set / 1024.0;
}

#endif  // CURIE_DEMOS_PROCESS_MEM_USAGE_H
