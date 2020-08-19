#include <xc.h>

.section .text.init_interrupt_controller,"ax"
.globl __init_interrupt_controller
.align 2
.ent __init_interrupt_controller
__init_interrupt_controller:
    li t0, 0xbf810540
    .extern __vector_offset_0
    la t1, __vector_offset_0
    sw t1, 0(t0)
    li t0, 0xbf810544
    .extern __vector_offset_1
    la t1, __vector_offset_1
    sw t1, 0(t0)
    li t0, 0xbf810548
    .extern __vector_offset_2
    la t1, __vector_offset_2
    sw t1, 0(t0)
    li t0, 0xbf81054c
    .extern __vector_offset_3
    la t1, __vector_offset_3
    sw t1, 0(t0)
    li t0, 0xbf810550
    .extern __vector_offset_4
    la t1, __vector_offset_4
    sw t1, 0(t0)
    li t0, 0xbf810554
    .extern __vector_offset_5
    la t1, __vector_offset_5
    sw t1, 0(t0)
    li t0, 0xbf810558
    .extern __vector_offset_6
    la t1, __vector_offset_6
    sw t1, 0(t0)
    li t0, 0xbf81055c
    .extern __vector_offset_7
    la t1, __vector_offset_7
    sw t1, 0(t0)
    li t0, 0xbf810560
    .extern __vector_offset_8
    la t1, __vector_offset_8
    sw t1, 0(t0)
    li t0, 0xbf810564
    .extern __vector_offset_9
    la t1, __vector_offset_9
    sw t1, 0(t0)
    li t0, 0xbf810568
    .extern __vector_offset_10
    la t1, __vector_offset_10
    sw t1, 0(t0)
    li t0, 0xbf81056c
    .extern __vector_offset_11
    la t1, __vector_offset_11
    sw t1, 0(t0)
    li t0, 0xbf810570
    .extern __vector_offset_12
    la t1, __vector_offset_12
    sw t1, 0(t0)
    li t0, 0xbf810574
    .extern __vector_offset_13
    la t1, __vector_offset_13
    sw t1, 0(t0)
    li t0, 0xbf810578
    .extern __vector_offset_14
    la t1, __vector_offset_14
    sw t1, 0(t0)
    li t0, 0xbf81057c
    .extern __vector_offset_15
    la t1, __vector_offset_15
    sw t1, 0(t0)
    li t0, 0xbf810580
    .extern __vector_offset_16
    la t1, __vector_offset_16
    sw t1, 0(t0)
    li t0, 0xbf810584
    .extern __vector_offset_17
    la t1, __vector_offset_17
    sw t1, 0(t0)
    li t0, 0xbf810588
    .extern __vector_offset_18
    la t1, __vector_offset_18
    sw t1, 0(t0)
    li t0, 0xbf81058c
    .extern __vector_offset_19
    la t1, __vector_offset_19
    sw t1, 0(t0)
    li t0, 0xbf810590
    .extern __vector_offset_20
    la t1, __vector_offset_20
    sw t1, 0(t0)
    li t0, 0xbf810594
    .extern __vector_offset_21
    la t1, __vector_offset_21
    sw t1, 0(t0)
    li t0, 0xbf810598
    .extern __vector_offset_22
    la t1, __vector_offset_22
    sw t1, 0(t0)
    li t0, 0xbf81059c
    .extern __vector_offset_23
    la t1, __vector_offset_23
    sw t1, 0(t0)
    li t0, 0xbf8105a0
    .extern __vector_offset_24
    la t1, __vector_offset_24
    sw t1, 0(t0)
    li t0, 0xbf8105a4
    .extern __vector_offset_25
    la t1, __vector_offset_25
    sw t1, 0(t0)
    li t0, 0xbf8105a8
    .extern __vector_offset_26
    la t1, __vector_offset_26
    sw t1, 0(t0)
    li t0, 0xbf8105ac
    .extern __vector_offset_27
    la t1, __vector_offset_27
    sw t1, 0(t0)
    li t0, 0xbf8105b0
    .extern __vector_offset_28
    la t1, __vector_offset_28
    sw t1, 0(t0)
    li t0, 0xbf8105b4
    .extern __vector_offset_29
    la t1, __vector_offset_29
    sw t1, 0(t0)
    li t0, 0xbf8105b8
    .extern __vector_offset_30
    la t1, __vector_offset_30
    sw t1, 0(t0)
    li t0, 0xbf8105bc
    .extern __vector_offset_31
    la t1, __vector_offset_31
    sw t1, 0(t0)
    li t0, 0xbf8105c0
    .extern __vector_offset_32
    la t1, __vector_offset_32
    sw t1, 0(t0)
    li t0, 0xbf8105c4
    .extern __vector_offset_33
    la t1, __vector_offset_33
    sw t1, 0(t0)
    li t0, 0xbf8105c8
    .extern __vector_offset_34
    la t1, __vector_offset_34
    sw t1, 0(t0)
    li t0, 0xbf8105cc
    .extern __vector_offset_35
    la t1, __vector_offset_35
    sw t1, 0(t0)
    li t0, 0xbf8105d0
    .extern __vector_offset_36
    la t1, __vector_offset_36
    sw t1, 0(t0)
    li t0, 0xbf8105d4
    .extern __vector_offset_37
    la t1, __vector_offset_37
    sw t1, 0(t0)
    li t0, 0xbf8105d8
    .extern __vector_offset_38
    la t1, __vector_offset_38
    sw t1, 0(t0)
    li t0, 0xbf8105dc
    .extern __vector_offset_39
    la t1, __vector_offset_39
    sw t1, 0(t0)
    li t0, 0xbf8105e0
    .extern __vector_offset_40
    la t1, __vector_offset_40
    sw t1, 0(t0)
    li t0, 0xbf8105e4
    .extern __vector_offset_41
    la t1, __vector_offset_41
    sw t1, 0(t0)
    li t0, 0xbf8105e8
    .extern __vector_offset_42
    la t1, __vector_offset_42
    sw t1, 0(t0)
    li t0, 0xbf8105ec
    .extern __vector_offset_43
    la t1, __vector_offset_43
    sw t1, 0(t0)
    li t0, 0xbf8105f0
    .extern __vector_offset_44
    la t1, __vector_offset_44
    sw t1, 0(t0)
    li t0, 0xbf8105f4
    .extern __vector_offset_45
    la t1, __vector_offset_45
    sw t1, 0(t0)
    li t0, 0xbf8105f8
    .extern __vector_offset_46
    la t1, __vector_offset_46
    sw t1, 0(t0)
    li t0, 0xbf8105fc
    .extern __vector_offset_47
    la t1, __vector_offset_47
    sw t1, 0(t0)
    li t0, 0xbf810600
    .extern __vector_offset_48
    la t1, __vector_offset_48
    sw t1, 0(t0)
    li t0, 0xbf810604
    .extern __vector_offset_49
    la t1, __vector_offset_49
    sw t1, 0(t0)
    li t0, 0xbf810608
    .extern __vector_offset_50
    la t1, __vector_offset_50
    sw t1, 0(t0)
    li t0, 0xbf81060c
    .extern __vector_offset_51
    la t1, __vector_offset_51
    sw t1, 0(t0)
    li t0, 0xbf810610
    .extern __vector_offset_52
    la t1, __vector_offset_52
    sw t1, 0(t0)
    li t0, 0xbf810614
    .extern __vector_offset_53
    la t1, __vector_offset_53
    sw t1, 0(t0)
    li t0, 0xbf810618
    .extern __vector_offset_54
    la t1, __vector_offset_54
    sw t1, 0(t0)
    li t0, 0xbf81061c
    .extern __vector_offset_55
    la t1, __vector_offset_55
    sw t1, 0(t0)
    li t0, 0xbf810620
    .extern __vector_offset_56
    la t1, __vector_offset_56
    sw t1, 0(t0)
    li t0, 0xbf810624
    .extern __vector_offset_57
    la t1, __vector_offset_57
    sw t1, 0(t0)
    li t0, 0xbf810628
    .extern __vector_offset_58
    la t1, __vector_offset_58
    sw t1, 0(t0)
    li t0, 0xbf81062c
    .extern __vector_offset_59
    la t1, __vector_offset_59
    sw t1, 0(t0)
    li t0, 0xbf810630
    .extern __vector_offset_60
    la t1, __vector_offset_60
    sw t1, 0(t0)
    li t0, 0xbf810634
    .extern __vector_offset_61
    la t1, __vector_offset_61
    sw t1, 0(t0)
    li t0, 0xbf810638
    .extern __vector_offset_62
    la t1, __vector_offset_62
    sw t1, 0(t0)
    li t0, 0xbf81063c
    .extern __vector_offset_63
    la t1, __vector_offset_63
    sw t1, 0(t0)
    li t0, 0xbf810640
    .extern __vector_offset_64
    la t1, __vector_offset_64
    sw t1, 0(t0)
    li t0, 0xbf810644
    .extern __vector_offset_65
    la t1, __vector_offset_65
    sw t1, 0(t0)
    li t0, 0xbf810648
    .extern __vector_offset_66
    la t1, __vector_offset_66
    sw t1, 0(t0)
    li t0, 0xbf81064c
    .extern __vector_offset_67
    la t1, __vector_offset_67
    sw t1, 0(t0)
    li t0, 0xbf810650
    .extern __vector_offset_68
    la t1, __vector_offset_68
    sw t1, 0(t0)
    li t0, 0xbf810654
    .extern __vector_offset_69
    la t1, __vector_offset_69
    sw t1, 0(t0)
    li t0, 0xbf810658
    .extern __vector_offset_70
    la t1, __vector_offset_70
    sw t1, 0(t0)
    li t0, 0xbf81065c
    .extern __vector_offset_71
    la t1, __vector_offset_71
    sw t1, 0(t0)
    li t0, 0xbf810660
    .extern __vector_offset_72
    la t1, __vector_offset_72
    sw t1, 0(t0)
    li t0, 0xbf810664
    .extern __vector_offset_73
    la t1, __vector_offset_73
    sw t1, 0(t0)
    li t0, 0xbf810668
    .extern __vector_offset_74
    la t1, __vector_offset_74
    sw t1, 0(t0)
    li t0, 0xbf81066c
    .extern __vector_offset_75
    la t1, __vector_offset_75
    sw t1, 0(t0)
    li t0, 0xbf810670
    .extern __vector_offset_76
    la t1, __vector_offset_76
    sw t1, 0(t0)
    li t0, 0xbf810674
    .extern __vector_offset_77
    la t1, __vector_offset_77
    sw t1, 0(t0)
    li t0, 0xbf810678
    .extern __vector_offset_78
    la t1, __vector_offset_78
    sw t1, 0(t0)
    li t0, 0xbf81067c
    .extern __vector_offset_79
    la t1, __vector_offset_79
    sw t1, 0(t0)
    li t0, 0xbf810680
    .extern __vector_offset_80
    la t1, __vector_offset_80
    sw t1, 0(t0)
    li t0, 0xbf810684
    .extern __vector_offset_81
    la t1, __vector_offset_81
    sw t1, 0(t0)
    li t0, 0xbf810688
    .extern __vector_offset_82
    la t1, __vector_offset_82
    sw t1, 0(t0)
    li t0, 0xbf81068c
    .extern __vector_offset_83
    la t1, __vector_offset_83
    sw t1, 0(t0)
    li t0, 0xbf810690
    .extern __vector_offset_84
    la t1, __vector_offset_84
    sw t1, 0(t0)
    li t0, 0xbf810694
    .extern __vector_offset_85
    la t1, __vector_offset_85
    sw t1, 0(t0)
    li t0, 0xbf810698
    .extern __vector_offset_86
    la t1, __vector_offset_86
    sw t1, 0(t0)
    li t0, 0xbf81069c
    .extern __vector_offset_87
    la t1, __vector_offset_87
    sw t1, 0(t0)
    li t0, 0xbf8106a0
    .extern __vector_offset_88
    la t1, __vector_offset_88
    sw t1, 0(t0)
    li t0, 0xbf8106a4
    .extern __vector_offset_89
    la t1, __vector_offset_89
    sw t1, 0(t0)
    li t0, 0xbf8106a8
    .extern __vector_offset_90
    la t1, __vector_offset_90
    sw t1, 0(t0)
    li t0, 0xbf8106ac
    .extern __vector_offset_91
    la t1, __vector_offset_91
    sw t1, 0(t0)
    li t0, 0xbf8106b0
    .extern __vector_offset_92
    la t1, __vector_offset_92
    sw t1, 0(t0)
    li t0, 0xbf8106b4
    .extern __vector_offset_93
    la t1, __vector_offset_93
    sw t1, 0(t0)
    li t0, 0xbf8106b8
    .extern __vector_offset_94
    la t1, __vector_offset_94
    sw t1, 0(t0)
    li t0, 0xbf8106bc
    .extern __vector_offset_95
    la t1, __vector_offset_95
    sw t1, 0(t0)
    li t0, 0xbf8106c0
    .extern __vector_offset_96
    la t1, __vector_offset_96
    sw t1, 0(t0)
    li t0, 0xbf8106c4
    .extern __vector_offset_97
    la t1, __vector_offset_97
    sw t1, 0(t0)
    li t0, 0xbf8106c8
    .extern __vector_offset_98
    la t1, __vector_offset_98
    sw t1, 0(t0)
    li t0, 0xbf8106cc
    .extern __vector_offset_99
    la t1, __vector_offset_99
    sw t1, 0(t0)
    li t0, 0xbf8106d0
    .extern __vector_offset_100
    la t1, __vector_offset_100
    sw t1, 0(t0)
    li t0, 0xbf8106d4
    .extern __vector_offset_101
    la t1, __vector_offset_101
    sw t1, 0(t0)
    li t0, 0xbf8106d8
    .extern __vector_offset_102
    la t1, __vector_offset_102
    sw t1, 0(t0)
    li t0, 0xbf8106dc
    .extern __vector_offset_103
    la t1, __vector_offset_103
    sw t1, 0(t0)
    li t0, 0xbf8106e0
    .extern __vector_offset_104
    la t1, __vector_offset_104
    sw t1, 0(t0)
    li t0, 0xbf8106e4
    .extern __vector_offset_105
    la t1, __vector_offset_105
    sw t1, 0(t0)
    li t0, 0xbf8106e8
    .extern __vector_offset_106
    la t1, __vector_offset_106
    sw t1, 0(t0)
    li t0, 0xbf8106ec
    .extern __vector_offset_107
    la t1, __vector_offset_107
    sw t1, 0(t0)
    li t0, 0xbf8106f0
    .extern __vector_offset_108
    la t1, __vector_offset_108
    sw t1, 0(t0)
    li t0, 0xbf8106f4
    .extern __vector_offset_109
    la t1, __vector_offset_109
    sw t1, 0(t0)
    li t0, 0xbf8106f8
    .extern __vector_offset_110
    la t1, __vector_offset_110
    sw t1, 0(t0)
    li t0, 0xbf8106fc
    .extern __vector_offset_111
    la t1, __vector_offset_111
    sw t1, 0(t0)
    li t0, 0xbf810700
    .extern __vector_offset_112
    la t1, __vector_offset_112
    sw t1, 0(t0)
    li t0, 0xbf810704
    .extern __vector_offset_113
    la t1, __vector_offset_113
    sw t1, 0(t0)
    li t0, 0xbf810708
    .extern __vector_offset_114
    la t1, __vector_offset_114
    sw t1, 0(t0)
    li t0, 0xbf81070c
    .extern __vector_offset_115
    la t1, __vector_offset_115
    sw t1, 0(t0)
    li t0, 0xbf810710
    .extern __vector_offset_116
    la t1, __vector_offset_116
    sw t1, 0(t0)
    li t0, 0xbf810714
    .extern __vector_offset_117
    la t1, __vector_offset_117
    sw t1, 0(t0)
    li t0, 0xbf810718
    .extern __vector_offset_118
    la t1, __vector_offset_118
    sw t1, 0(t0)
    li t0, 0xbf81071c
    .extern __vector_offset_119
    la t1, __vector_offset_119
    sw t1, 0(t0)
    li t0, 0xbf810720
    .extern __vector_offset_120
    la t1, __vector_offset_120
    sw t1, 0(t0)
    li t0, 0xbf810724
    .extern __vector_offset_121
    la t1, __vector_offset_121
    sw t1, 0(t0)
    li t0, 0xbf810728
    .extern __vector_offset_122
    la t1, __vector_offset_122
    sw t1, 0(t0)
    li t0, 0xbf81072c
    .extern __vector_offset_123
    la t1, __vector_offset_123
    sw t1, 0(t0)
    li t0, 0xbf810730
    .extern __vector_offset_124
    la t1, __vector_offset_124
    sw t1, 0(t0)
    li t0, 0xbf810734
    .extern __vector_offset_125
    la t1, __vector_offset_125
    sw t1, 0(t0)
    li t0, 0xbf810738
    .extern __vector_offset_126
    la t1, __vector_offset_126
    sw t1, 0(t0)
    li t0, 0xbf81073c
    .extern __vector_offset_127
    la t1, __vector_offset_127
    sw t1, 0(t0)
    li t0, 0xbf810740
    .extern __vector_offset_128
    la t1, __vector_offset_128
    sw t1, 0(t0)
    li t0, 0xbf810744
    .extern __vector_offset_129
    la t1, __vector_offset_129
    sw t1, 0(t0)
    li t0, 0xbf810748
    .extern __vector_offset_130
    la t1, __vector_offset_130
    sw t1, 0(t0)
    li t0, 0xbf81074c
    .extern __vector_offset_131
    la t1, __vector_offset_131
    sw t1, 0(t0)
    li t0, 0xbf810750
    .extern __vector_offset_132
    la t1, __vector_offset_132
    sw t1, 0(t0)
    li t0, 0xbf810754
    .extern __vector_offset_133
    la t1, __vector_offset_133
    sw t1, 0(t0)
    li t0, 0xbf810758
    .extern __vector_offset_134
    la t1, __vector_offset_134
    sw t1, 0(t0)
    li t0, 0xbf81075c
    .extern __vector_offset_135
    la t1, __vector_offset_135
    sw t1, 0(t0)
    li t0, 0xbf810760
    .extern __vector_offset_136
    la t1, __vector_offset_136
    sw t1, 0(t0)
    li t0, 0xbf810764
    .extern __vector_offset_137
    la t1, __vector_offset_137
    sw t1, 0(t0)
    li t0, 0xbf810768
    .extern __vector_offset_138
    la t1, __vector_offset_138
    sw t1, 0(t0)
    li t0, 0xbf81076c
    .extern __vector_offset_139
    la t1, __vector_offset_139
    sw t1, 0(t0)
    li t0, 0xbf810770
    .extern __vector_offset_140
    la t1, __vector_offset_140
    sw t1, 0(t0)
    li t0, 0xbf810774
    .extern __vector_offset_141
    la t1, __vector_offset_141
    sw t1, 0(t0)
    li t0, 0xbf810778
    .extern __vector_offset_142
    la t1, __vector_offset_142
    sw t1, 0(t0)
    li t0, 0xbf81077c
    .extern __vector_offset_143
    la t1, __vector_offset_143
    sw t1, 0(t0)
    li t0, 0xbf810780
    .extern __vector_offset_144
    la t1, __vector_offset_144
    sw t1, 0(t0)
    li t0, 0xbf810784
    .extern __vector_offset_145
    la t1, __vector_offset_145
    sw t1, 0(t0)
    li t0, 0xbf810788
    .extern __vector_offset_146
    la t1, __vector_offset_146
    sw t1, 0(t0)
    li t0, 0xbf81078c
    .extern __vector_offset_147
    la t1, __vector_offset_147
    sw t1, 0(t0)
    li t0, 0xbf810790
    .extern __vector_offset_148
    la t1, __vector_offset_148
    sw t1, 0(t0)
    li t0, 0xbf810794
    .extern __vector_offset_149
    la t1, __vector_offset_149
    sw t1, 0(t0)
    li t0, 0xbf810798
    .extern __vector_offset_150
    la t1, __vector_offset_150
    sw t1, 0(t0)
    li t0, 0xbf81079c
    .extern __vector_offset_151
    la t1, __vector_offset_151
    sw t1, 0(t0)
    li t0, 0xbf8107a0
    .extern __vector_offset_152
    la t1, __vector_offset_152
    sw t1, 0(t0)
    li t0, 0xbf8107a4
    .extern __vector_offset_153
    la t1, __vector_offset_153
    sw t1, 0(t0)
    li t0, 0xbf8107a8
    .extern __vector_offset_154
    la t1, __vector_offset_154
    sw t1, 0(t0)
    li t0, 0xbf8107ac
    .extern __vector_offset_155
    la t1, __vector_offset_155
    sw t1, 0(t0)
    li t0, 0xbf8107b0
    .extern __vector_offset_156
    la t1, __vector_offset_156
    sw t1, 0(t0)
    li t0, 0xbf8107b4
    .extern __vector_offset_157
    la t1, __vector_offset_157
    sw t1, 0(t0)
    li t0, 0xbf8107b8
    .extern __vector_offset_158
    la t1, __vector_offset_158
    sw t1, 0(t0)
    li t0, 0xbf8107bc
    .extern __vector_offset_159
    la t1, __vector_offset_159
    sw t1, 0(t0)
    li t0, 0xbf8107c0
    .extern __vector_offset_160
    la t1, __vector_offset_160
    sw t1, 0(t0)
    li t0, 0xbf8107c4
    .extern __vector_offset_161
    la t1, __vector_offset_161
    sw t1, 0(t0)
    li t0, 0xbf8107c8
    .extern __vector_offset_162
    la t1, __vector_offset_162
    sw t1, 0(t0)
    li t0, 0xbf8107cc
    .extern __vector_offset_163
    la t1, __vector_offset_163
    sw t1, 0(t0)
    li t0, 0xbf8107d0
    .extern __vector_offset_164
    la t1, __vector_offset_164
    sw t1, 0(t0)
    li t0, 0xbf8107d4
    .extern __vector_offset_165
    la t1, __vector_offset_165
    sw t1, 0(t0)
    li t0, 0xbf8107d8
    .extern __vector_offset_166
    la t1, __vector_offset_166
    sw t1, 0(t0)
    li t0, 0xbf8107dc
    .extern __vector_offset_167
    la t1, __vector_offset_167
    sw t1, 0(t0)
    li t0, 0xbf8107e0
    .extern __vector_offset_168
    la t1, __vector_offset_168
    sw t1, 0(t0)
    li t0, 0xbf8107e4
    .extern __vector_offset_169
    la t1, __vector_offset_169
    sw t1, 0(t0)
    li t0, 0xbf8107e8
    .extern __vector_offset_170
    la t1, __vector_offset_170
    sw t1, 0(t0)
    li t0, 0xbf8107ec
    .extern __vector_offset_171
    la t1, __vector_offset_171
    sw t1, 0(t0)
    li t0, 0xbf8107f0
    .extern __vector_offset_172
    la t1, __vector_offset_172
    sw t1, 0(t0)
    li t0, 0xbf8107f4
    .extern __vector_offset_173
    la t1, __vector_offset_173
    sw t1, 0(t0)
    li t0, 0xbf8107f8
    .extern __vector_offset_174
    la t1, __vector_offset_174
    sw t1, 0(t0)
    li t0, 0xbf8107fc
    .extern __vector_offset_175
    la t1, __vector_offset_175
    sw t1, 0(t0)
    li t0, 0xbf810800
    .extern __vector_offset_176
    la t1, __vector_offset_176
    sw t1, 0(t0)
    li t0, 0xbf810804
    .extern __vector_offset_177
    la t1, __vector_offset_177
    sw t1, 0(t0)
    li t0, 0xbf810808
    .extern __vector_offset_178
    la t1, __vector_offset_178
    sw t1, 0(t0)
    li t0, 0xbf81080c
    .extern __vector_offset_179
    la t1, __vector_offset_179
    sw t1, 0(t0)
    li t0, 0xbf810810
    .extern __vector_offset_180
    la t1, __vector_offset_180
    sw t1, 0(t0)
    li t0, 0xbf810814
    .extern __vector_offset_181
    la t1, __vector_offset_181
    sw t1, 0(t0)
    li t0, 0xbf810818
    .extern __vector_offset_182
    la t1, __vector_offset_182
    sw t1, 0(t0)
    li t0, 0xbf81081c
    .extern __vector_offset_183
    la t1, __vector_offset_183
    sw t1, 0(t0)
    li t0, 0xbf810820
    .extern __vector_offset_184
    la t1, __vector_offset_184
    sw t1, 0(t0)
    li t0, 0xbf810824
    .extern __vector_offset_185
    la t1, __vector_offset_185
    sw t1, 0(t0)
    li t0, 0xbf810828
    .extern __vector_offset_186
    la t1, __vector_offset_186
    sw t1, 0(t0)
    li t0, 0xbf81082c
    .extern __vector_offset_187
    la t1, __vector_offset_187
    sw t1, 0(t0)
    li t0, 0xbf810830
    .extern __vector_offset_188
    la t1, __vector_offset_188
    sw t1, 0(t0)
    li t0, 0xbf810834
    .extern __vector_offset_189
    la t1, __vector_offset_189
    sw t1, 0(t0)
    li t0, 0xbf810838
    .extern __vector_offset_190
    la t1, __vector_offset_190
    sw t1, 0(t0)
    li t0, 0xbf81083c
    .extern __vector_offset_191
    la t1, __vector_offset_191
    sw t1, 0(t0)
    li t0, 0xbf810840
    .extern __vector_offset_192
    la t1, __vector_offset_192
    sw t1, 0(t0)
    li t0, 0xbf810844
    .extern __vector_offset_193
    la t1, __vector_offset_193
    sw t1, 0(t0)
    li t0, 0xbf810848
    .extern __vector_offset_194
    la t1, __vector_offset_194
    sw t1, 0(t0)
    li t0, 0xbf81084c
    .extern __vector_offset_195
    la t1, __vector_offset_195
    sw t1, 0(t0)
    li t0, 0xbf810850
    .extern __vector_offset_196
    la t1, __vector_offset_196
    sw t1, 0(t0)
    li t0, 0xbf810854
    .extern __vector_offset_197
    la t1, __vector_offset_197
    sw t1, 0(t0)
    li t0, 0xbf810858
    .extern __vector_offset_198
    la t1, __vector_offset_198
    sw t1, 0(t0)
    li t0, 0xbf81085c
    .extern __vector_offset_199
    la t1, __vector_offset_199
    sw t1, 0(t0)
    li t0, 0xbf810860
    .extern __vector_offset_200
    la t1, __vector_offset_200
    sw t1, 0(t0)
    li t0, 0xbf810864
    .extern __vector_offset_201
    la t1, __vector_offset_201
    sw t1, 0(t0)
    li t0, 0xbf810868
    .extern __vector_offset_202
    la t1, __vector_offset_202
    sw t1, 0(t0)
    li t0, 0xbf81086c
    .extern __vector_offset_203
    la t1, __vector_offset_203
    sw t1, 0(t0)
    li t0, 0xbf810870
    .extern __vector_offset_204
    la t1, __vector_offset_204
    sw t1, 0(t0)
    li t0, 0xbf810874
    .extern __vector_offset_205
    la t1, __vector_offset_205
    sw t1, 0(t0)
    li t0, 0xbf810878
    .extern __vector_offset_206
    la t1, __vector_offset_206
    sw t1, 0(t0)
    li t0, 0xbf81087c
    .extern __vector_offset_207
    la t1, __vector_offset_207
    sw t1, 0(t0)
    li t0, 0xbf810880
    .extern __vector_offset_208
    la t1, __vector_offset_208
    sw t1, 0(t0)
    li t0, 0xbf810884
    .extern __vector_offset_209
    la t1, __vector_offset_209
    sw t1, 0(t0)
    li t0, 0xbf810888
    .extern __vector_offset_210
    la t1, __vector_offset_210
    sw t1, 0(t0)
    li t0, 0xbf81088c
    .extern __vector_offset_211
    la t1, __vector_offset_211
    sw t1, 0(t0)
    li t0, 0xbf810890
    .extern __vector_offset_212
    la t1, __vector_offset_212
    sw t1, 0(t0)
    li t0, 0xbf810894
    .extern __vector_offset_213
    la t1, __vector_offset_213
    sw t1, 0(t0)
    li t0, 0xbf810898
    .extern __vector_offset_214
    la t1, __vector_offset_214
    sw t1, 0(t0)
    li t0, 0xbf81089c
    .extern __vector_offset_215
    la t1, __vector_offset_215
    sw t1, 0(t0)
    li t0, 0xbf8108a0
    .extern __vector_offset_216
    la t1, __vector_offset_216
    sw t1, 0(t0)
    li t0, 0xbf8108a4
    .extern __vector_offset_217
    la t1, __vector_offset_217
    sw t1, 0(t0)
    li t0, 0xbf8108a8
    .extern __vector_offset_218
    la t1, __vector_offset_218
    sw t1, 0(t0)
    li t0, 0xbf8108ac
    .extern __vector_offset_219
    la t1, __vector_offset_219
    sw t1, 0(t0)
    li t0, 0xbf8108b0
    .extern __vector_offset_220
    la t1, __vector_offset_220
    sw t1, 0(t0)
    li t0, 0xbf8108b4
    .extern __vector_offset_221
    la t1, __vector_offset_221
    sw t1, 0(t0)
    li t0, 0xbf8108b8
    .extern __vector_offset_222
    la t1, __vector_offset_222
    sw t1, 0(t0)
    li t0, 0xbf8108bc
    .extern __vector_offset_223
    la t1, __vector_offset_223
    sw t1, 0(t0)
    li t0, 0xbf8108c0
    .extern __vector_offset_224
    la t1, __vector_offset_224
    sw t1, 0(t0)
    li t0, 0xbf8108c4
    .extern __vector_offset_225
    la t1, __vector_offset_225
    sw t1, 0(t0)
    li t0, 0xbf8108c8
    .extern __vector_offset_226
    la t1, __vector_offset_226
    sw t1, 0(t0)
    li t0, 0xbf8108cc
    .extern __vector_offset_227
    la t1, __vector_offset_227
    sw t1, 0(t0)
    li t0, 0xbf8108d0
    .extern __vector_offset_228
    la t1, __vector_offset_228
    sw t1, 0(t0)
    li t0, 0xbf8108d4
    .extern __vector_offset_229
    la t1, __vector_offset_229
    sw t1, 0(t0)
    li t0, 0xbf8108d8
    .extern __vector_offset_230
    la t1, __vector_offset_230
    sw t1, 0(t0)
    li t0, 0xbf8108dc
    .extern __vector_offset_231
    la t1, __vector_offset_231
    sw t1, 0(t0)
    li t0, 0xbf8108e0
    .extern __vector_offset_232
    la t1, __vector_offset_232
    sw t1, 0(t0)
    li t0, 0xbf8108e4
    .extern __vector_offset_233
    la t1, __vector_offset_233
    sw t1, 0(t0)
    li t0, 0xbf8108e8
    .extern __vector_offset_234
    la t1, __vector_offset_234
    sw t1, 0(t0)
    li t0, 0xbf8108ec
    .extern __vector_offset_235
    la t1, __vector_offset_235
    sw t1, 0(t0)
    li t0, 0xbf8108f0
    .extern __vector_offset_236
    la t1, __vector_offset_236
    sw t1, 0(t0)
    li t0, 0xbf8108f4
    .extern __vector_offset_237
    la t1, __vector_offset_237
    sw t1, 0(t0)
    li t0, 0xbf8108f8
    .extern __vector_offset_238
    la t1, __vector_offset_238
    sw t1, 0(t0)
    li t0, 0xbf8108fc
    .extern __vector_offset_239
    la t1, __vector_offset_239
    sw t1, 0(t0)
    li t0, 0xbf810900
    .extern __vector_offset_240
    la t1, __vector_offset_240
    sw t1, 0(t0)
    li t0, 0xbf810904
    .extern __vector_offset_241
    la t1, __vector_offset_241
    sw t1, 0(t0)
    li t0, 0xbf810908
    .extern __vector_offset_242
    la t1, __vector_offset_242
    sw t1, 0(t0)
    li t0, 0xbf81090c
    .extern __vector_offset_243
    la t1, __vector_offset_243
    sw t1, 0(t0)
    li t0, 0xbf810910
    .extern __vector_offset_244
    la t1, __vector_offset_244
    sw t1, 0(t0)
    li t0, 0xbf810914
    .extern __vector_offset_245
    la t1, __vector_offset_245
    sw t1, 0(t0)
    li t0, 0xbf810918
    .extern __vector_offset_246
    la t1, __vector_offset_246
    sw t1, 0(t0)
    li t0, 0xbf81091c
    .extern __vector_offset_247
    la t1, __vector_offset_247
    sw t1, 0(t0)
    li t0, 0xbf810920
    .extern __vector_offset_248
    la t1, __vector_offset_248
    sw t1, 0(t0)
    li t0, 0xbf810924
    .extern __vector_offset_249
    la t1, __vector_offset_249
    sw t1, 0(t0)
    li t0, 0xbf810928
    .extern __vector_offset_250
    la t1, __vector_offset_250
    sw t1, 0(t0)
    li t0, 0xbf81092c
    .extern __vector_offset_251
    la t1, __vector_offset_251
    sw t1, 0(t0)
    li t0, 0xbf810930
    .extern __vector_offset_252
    la t1, __vector_offset_252
    sw t1, 0(t0)
    li t0, 0xbf810934
    .extern __vector_offset_253
    la t1, __vector_offset_253
    sw t1, 0(t0)
    li t0, 0xbf810938
    .extern __vector_offset_254
    la t1, __vector_offset_254
    sw t1, 0(t0)
    jr ra
    nop
.end __init_interrupt_controller
