.. SPDX-License-Identifier: GPL-2.0

========================================
STM32 DDR Performance Monitor (DDRPERFM)
========================================

The DDRPERFM is the DDR Performance Monitor embedded in STM32MPU SoC.
The DDR controller provides events to DDRPERFM, once selected they are counted in the DDRPERFM
peripheral.

In MP1 family, the DDRPERFM is able to count 4 different events at the same time.
However, the 4 events must belong to the same set.
One hardware counter is dedicated to the time counter, `time_cnt`.

In MP2 family, the DDRPERFM is able to select between 44 different DDR events.
As for MP1, there is a dedicated hardware counter for the time.
It is incremented every 4 DDR clock cycles.
All the other counters can be freely allocated to count any other DDR event.

The stm32-ddr-pmu driver relies on the perf PMU framework to expose the counters via sysfs:

On MP1:

    .. code-block:: bash

        $ ls /sys/bus/event_source/devices/stm32_ddr_pmu/events/
        cactive_ddrc                 perf_lpr_req_with_no_credit  perf_op_is_wr
        ctl_idle                     perf_lpr_xact_when_critical  perf_selfresh_mode
        dfi_lp_req                   perf_op_is_activate          perf_wr_xact_when_critical
        dfi_lp_req_cpy               perf_op_is_enter_powerdown   time_cnt
        perf_hpr_req_with_no_credit  perf_op_is_rd
        perf_hpr_xact_when_critical  perf_op_is_refresh

On MP2:

    .. code-block:: bash

        $ ls /sys/bus/event_source/devices/stm32_ddr_pmu/events/
        dfi_is_act       perf_hpr_req_with_nocredit   perf_op_is_spec_ref
        dfi_is_mpc       perf_hpr_xact_when_critical  perf_op_is_wr
        dfi_is_mrr       perf_lpr_req_with_nocredit   perf_op_is_zqcal
        dfi_is_mrw       perf_lpr_xact_when_critical  perf_rank
        dfi_is_mwr       perf_op_is_act               perf_raw_hazard
        dfi_is_mwra      perf_op_is_crit_ref          perf_rdwr_transitions
        dfi_is_preab     perf_op_is_enter_powdn       perf_read_bypass
        dfi_is_prepb     perf_op_is_enter_selfref     perf_war_hazard
        dfi_is_rd        perf_op_is_mwr               perf_waw_hazard
        dfi_is_rda       perf_op_is_pre               perf_window_limit_reached_rd
        dfi_is_refab     perf_op_is_pre_for_others    perf_window_limit_reached_wr
        dfi_is_refpb     perf_op_is_pre_for_rdwr      perf_wr_xact_when_critical
        dfi_is_wr        perf_op_is_rd                time_cnt
        dfi_is_wra       perf_op_is_rd_activate
        perf_act_bypass  perf_op_is_ref


The perf PMU framework is usually invoked via the 'perf stat' tool.


Example:

    .. code-block:: bash

        $ perf stat --timeout 60000 -e stm32_ddr_pmu/dfi_is_act/,\
        > stm32_ddr_pmu/dfi_is_rd/,\
        > stm32_ddr_pmu/dfi_is_wr/,\
        > stm32_ddr_pmu/dfi_is_refab/,\
        > stm32_ddr_pmu/dfi_is_mrw/,\
        > stm32_ddr_pmu/dfi_is_rda/,\
        > stm32_ddr_pmu/dfi_is_wra/,\
        > stm32_ddr_pmu/dfi_is_mrr/,\
        > stm32_ddr_pmu/time_cnt/ \
        > -a sleep 5

         Performance counter stats for 'system wide':

                    481025      stm32_ddr_pmu/dfi_is_act/
                    732166      stm32_ddr_pmu/dfi_is_rd/
                    144926      stm32_ddr_pmu/dfi_is_wr/
                    644154      stm32_ddr_pmu/dfi_is_refab/
                         0      stm32_ddr_pmu/dfi_is_mrw/
                         0      stm32_ddr_pmu/dfi_is_rda/
                         0      stm32_ddr_pmu/dfi_is_wra/
                         0      stm32_ddr_pmu/dfi_is_mrr/
                 752347686      stm32_ddr_pmu/time_cnt/

               5.014910750 seconds time elapsed
