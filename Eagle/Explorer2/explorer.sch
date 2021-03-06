<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="7.5.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="quadcopter_comp">
<packages>
<package name="MOTOR_HEADER">
<wire x1="-3.81" y1="5.08" x2="-3.81" y2="-5.08" width="0.127" layer="21"/>
<wire x1="-3.81" y1="-5.08" x2="3.81" y2="-5.08" width="0.127" layer="21"/>
<wire x1="3.81" y1="-5.08" x2="3.81" y2="5.08" width="0.127" layer="21"/>
<wire x1="3.81" y1="5.08" x2="-3.81" y2="5.08" width="0.127" layer="21"/>
<pad name="FL" x="-2.54" y="3.81" drill="0.8" diameter="1.778"/>
<pad name="GND1" x="2.54" y="3.81" drill="0.8" diameter="1.778"/>
<pad name="FR" x="-2.54" y="1.27" drill="0.8" diameter="1.778"/>
<pad name="BL" x="-2.54" y="-1.27" drill="0.8" diameter="1.778"/>
<pad name="BR" x="-2.54" y="-3.81" drill="0.8" diameter="1.778"/>
<pad name="GND4" x="2.54" y="-3.81" drill="0.8" diameter="1.778"/>
<pad name="GND3" x="2.54" y="-1.27" drill="0.8" diameter="1.778"/>
<pad name="GND2" x="2.54" y="1.27" drill="0.8" diameter="1.778"/>
</package>
<package name="SAMD21G-MINI">
<wire x1="8.89" y1="-16.51" x2="-8.89" y2="-16.51" width="0.127" layer="21"/>
<wire x1="-8.89" y1="-16.51" x2="-8.89" y2="16.51" width="0.127" layer="21"/>
<wire x1="-8.89" y1="16.51" x2="8.89" y2="16.51" width="0.127" layer="21"/>
<wire x1="8.89" y1="16.51" x2="8.89" y2="-16.51" width="0.127" layer="21"/>
<pad name="9" x="-7.62" y="-15.24" drill="1" diameter="1.9304"/>
<pad name="8" x="-7.62" y="-12.7" drill="1" diameter="1.9304"/>
<pad name="7" x="-7.62" y="-10.16" drill="1" diameter="1.9304"/>
<pad name="6" x="-7.62" y="-7.62" drill="1" diameter="1.9304"/>
<pad name="5" x="-7.62" y="-5.08" drill="1" diameter="1.9304"/>
<pad name="4" x="-7.62" y="-2.54" drill="1" diameter="1.9304"/>
<pad name="3" x="-7.62" y="0" drill="1" diameter="1.9304"/>
<pad name="2" x="-7.62" y="2.54" drill="1" diameter="1.9304"/>
<pad name="GND" x="-7.62" y="5.08" drill="1" diameter="1.9304"/>
<pad name="RSTN" x="-7.62" y="7.62" drill="1" diameter="1.9304"/>
<pad name="0/RX" x="-7.62" y="10.16" drill="1" diameter="1.9304"/>
<pad name="1/RX" x="-7.62" y="12.7" drill="1" diameter="1.9304"/>
<pad name="VIN" x="7.62" y="12.7" drill="1" diameter="1.9304"/>
<pad name="GND2" x="7.62" y="10.16" drill="1" diameter="1.9304"/>
<pad name="AREF" x="7.62" y="7.62" drill="1" diameter="1.9304"/>
<pad name="VCC" x="7.62" y="5.08" drill="1" diameter="1.9304"/>
<pad name="A3" x="7.62" y="2.54" drill="1" diameter="1.9304"/>
<pad name="A2" x="7.62" y="0" drill="1" diameter="1.9304"/>
<pad name="A1" x="7.62" y="-2.54" drill="1" diameter="1.9304"/>
<pad name="A0" x="7.62" y="-5.08" drill="1" diameter="1.9304"/>
<pad name="13" x="7.62" y="-7.62" drill="1" diameter="1.9304"/>
<pad name="12" x="7.62" y="-10.16" drill="1" diameter="1.9304"/>
<pad name="11" x="7.62" y="-12.7" drill="1" diameter="1.9304"/>
<pad name="10" x="7.62" y="-15.24" drill="1" diameter="1.9304"/>
<text x="-6.35" y="12.7" size="0.8128" layer="35" align="center-left">1/TX</text>
<text x="-6.35" y="-15.24" size="0.8128" layer="35" align="center-left">9</text>
<text x="6.35" y="-15.24" size="0.8128" layer="35" align="center-right">10</text>
<text x="-6.35" y="-12.7" size="0.8128" layer="35" align="center-left">8</text>
<text x="-6.35" y="-10.16" size="0.8128" layer="35" align="center-left">7</text>
<text x="-6.35" y="-7.62" size="0.8128" layer="35" align="center-left">6</text>
<text x="-6.35" y="-5.08" size="0.8128" layer="35" align="center-left">5</text>
<text x="-6.35" y="-2.54" size="0.8128" layer="35" align="center-left">4</text>
<text x="-6.35" y="0" size="0.8128" layer="35" align="center-left">3</text>
<text x="-6.35" y="2.54" size="0.8128" layer="35" align="center-left">2</text>
<text x="-6.35" y="5.08" size="0.8128" layer="35" align="center-left">GND</text>
<text x="-6.35" y="7.62" size="0.8128" layer="35" align="center-left">RSTn</text>
<text x="-6.35" y="10.16" size="0.8128" layer="35" align="center-left">0/RX</text>
<text x="6.35" y="-12.7" size="0.8128" layer="35" align="center-right">11</text>
<text x="6.35" y="-10.16" size="0.8128" layer="35" align="center-right">12</text>
<text x="6.35" y="-7.62" size="0.8128" layer="35" align="center-right">13</text>
<text x="6.35" y="-5.08" size="0.8128" layer="35" align="center-right">A0</text>
<text x="6.35" y="-2.54" size="0.8128" layer="35" align="center-right">A1</text>
<text x="6.35" y="0" size="0.8128" layer="35" align="center-right">A2</text>
<text x="6.35" y="2.54" size="0.8128" layer="35" align="center-right">A3</text>
<text x="6.35" y="5.08" size="0.8128" layer="35" align="center-right">VCC</text>
<text x="6.35" y="7.62" size="0.8128" layer="35" align="center-right">AREF</text>
<text x="6.35" y="10.16" size="0.8128" layer="35" align="center-right">GND</text>
<text x="6.35" y="12.7" size="0.8128" layer="35" align="center-right">VIN</text>
<pad name="SCL" x="3.81" y="3.81" drill="1" diameter="1.9304"/>
<pad name="SDA" x="3.81" y="1.27" drill="1" diameter="1.9304"/>
<text x="2.54" y="3.81" size="0.8128" layer="35" align="center-right">SCL</text>
<text x="2.54" y="1.27" size="0.8128" layer="35" align="center-right">SDA</text>
<wire x1="-3.81" y1="16.51" x2="-3.81" y2="11.43" width="0.127" layer="35"/>
<wire x1="-3.81" y1="11.43" x2="3.81" y2="11.43" width="0.127" layer="35"/>
<wire x1="3.81" y1="11.43" x2="3.81" y2="16.51" width="0.127" layer="35"/>
</package>
<package name="LEVELSHIFT_8CH">
<pad name="OE" x="-6.35" y="-11.43" drill="1.1" diameter="1.9304"/>
<pad name="A8" x="-6.35" y="-8.89" drill="1.1" diameter="1.9304"/>
<pad name="B8" x="6.35" y="-8.89" drill="1.1" diameter="1.9304"/>
<pad name="GND" x="6.35" y="-11.43" drill="1.1" diameter="1.9304"/>
<pad name="A7" x="-6.35" y="-6.35" drill="1.1" diameter="1.9304"/>
<pad name="B7" x="6.35" y="-6.35" drill="1.1" diameter="1.9304"/>
<pad name="B6" x="6.35" y="-3.81" drill="1.1" diameter="1.9304"/>
<pad name="A6" x="-6.35" y="-3.81" drill="1.1" diameter="1.9304"/>
<pad name="B5" x="6.35" y="-1.27" drill="1.1" diameter="1.9304"/>
<pad name="B4" x="6.35" y="1.27" drill="1.1" diameter="1.9304"/>
<pad name="B3" x="6.35" y="3.81" drill="1.1" diameter="1.9304"/>
<pad name="B2" x="6.35" y="6.35" drill="1.1" diameter="1.9304"/>
<pad name="B1" x="6.35" y="8.89" drill="1.1" diameter="1.9304"/>
<pad name="VCCB" x="6.35" y="11.43" drill="1.1" diameter="1.9304"/>
<pad name="A5" x="-6.35" y="-1.27" drill="1.1" diameter="1.9304"/>
<pad name="A4" x="-6.35" y="1.27" drill="1.1" diameter="1.9304"/>
<pad name="A3" x="-6.35" y="3.81" drill="1.1" diameter="1.9304"/>
<pad name="A2" x="-6.35" y="6.35" drill="1.1" diameter="1.9304"/>
<pad name="A1" x="-6.35" y="8.89" drill="1.1" diameter="1.9304"/>
<pad name="VCCA" x="-6.35" y="11.43" drill="1.1" diameter="1.9304"/>
<wire x1="7.62" y1="-13.97" x2="7.62" y2="13.97" width="0.127" layer="21"/>
<wire x1="7.62" y1="13.97" x2="-7.62" y2="13.97" width="0.127" layer="21"/>
<wire x1="-7.62" y1="13.97" x2="-7.62" y2="-13.97" width="0.127" layer="21"/>
<wire x1="-7.62" y1="-13.97" x2="7.62" y2="-13.97" width="0.127" layer="21"/>
</package>
<package name="RADIO_8CH">
<wire x1="-2.54" y1="-7.62" x2="-2.54" y2="7.62" width="0.127" layer="21"/>
<wire x1="-2.54" y1="7.62" x2="2.54" y2="7.62" width="0.127" layer="21"/>
<wire x1="2.54" y1="7.62" x2="2.54" y2="-7.62" width="0.127" layer="21"/>
<wire x1="2.54" y1="-7.62" x2="-2.54" y2="-7.62" width="0.127" layer="21"/>
<pad name="CH3" x="-1.27" y="3.81" drill="0.8" diameter="1.778"/>
<pad name="CH4" x="1.27" y="3.81" drill="0.8" diameter="1.778"/>
<pad name="CH5" x="-1.27" y="1.27" drill="0.8" diameter="1.778"/>
<pad name="CH6" x="1.27" y="1.27" drill="0.8" diameter="1.778"/>
<pad name="CH7" x="-1.27" y="-1.27" drill="0.8" diameter="1.778"/>
<pad name="VCC" x="-1.27" y="-6.35" drill="0.8" diameter="1.778"/>
<pad name="CH2" x="1.27" y="6.35" drill="0.8" diameter="1.778"/>
<pad name="CH8" x="1.27" y="-1.27" drill="0.8" diameter="1.778"/>
<pad name="CH1" x="-1.27" y="6.35" drill="0.8" diameter="1.778"/>
<pad name="GND" x="1.27" y="-6.35" drill="0.8" diameter="1.778"/>
</package>
<package name="LEVELSHIFT_4CH">
<pad name="GND2" x="5.08" y="-6.35" drill="1.1" diameter="1.9304"/>
<pad name="B4" x="5.08" y="-3.81" drill="1.1" diameter="1.9304"/>
<pad name="B3" x="5.08" y="-1.27" drill="1.1" diameter="1.9304"/>
<pad name="B2" x="5.08" y="1.27" drill="1.1" diameter="1.9304"/>
<pad name="B1" x="5.08" y="3.81" drill="1.1" diameter="1.9304"/>
<pad name="HV" x="5.08" y="6.35" drill="1.1" diameter="1.9304"/>
<pad name="GND" x="-5.08" y="-6.35" drill="1.1" diameter="1.9304"/>
<pad name="A4" x="-5.08" y="-3.81" drill="1.1" diameter="1.9304"/>
<pad name="A3" x="-5.08" y="-1.27" drill="1.1" diameter="1.9304"/>
<pad name="A2" x="-5.08" y="1.27" drill="1.1" diameter="1.9304"/>
<pad name="A1" x="-5.08" y="3.81" drill="1.1" diameter="1.9304"/>
<pad name="LV" x="-5.08" y="6.35" drill="1.1" diameter="1.9304"/>
<wire x1="6.35" y1="-10.16" x2="6.35" y2="10.16" width="0.127" layer="21"/>
<wire x1="6.35" y1="10.16" x2="-6.35" y2="10.16" width="0.127" layer="21"/>
<wire x1="-6.35" y1="10.16" x2="-6.35" y2="-10.16" width="0.127" layer="21"/>
<wire x1="-6.35" y1="-10.16" x2="6.35" y2="-10.16" width="0.127" layer="21"/>
</package>
<package name="GY-86">
<wire x1="-7.62" y1="10.16" x2="-7.62" y2="-10.16" width="0.127" layer="21"/>
<wire x1="-7.62" y1="-10.16" x2="7.62" y2="-10.16" width="0.127" layer="21"/>
<wire x1="7.62" y1="-10.16" x2="7.62" y2="10.16" width="0.127" layer="21"/>
<wire x1="7.62" y1="10.16" x2="-7.62" y2="10.16" width="0.127" layer="21"/>
<pad name="VCC" x="-6.35" y="8.89" drill="0.8" diameter="1.778"/>
<pad name="3V3" x="-6.35" y="6.35" drill="0.8" diameter="1.778"/>
<pad name="GND" x="-6.35" y="3.81" drill="0.8" diameter="1.778"/>
<pad name="SCL" x="-6.35" y="1.27" drill="0.8" diameter="1.778"/>
<pad name="SDA" x="-6.35" y="-1.27" drill="0.8" diameter="1.778"/>
<pad name="FSYNC" x="-6.35" y="-3.81" drill="0.8" diameter="1.778"/>
<pad name="INTA" x="-6.35" y="-6.35" drill="0.8" diameter="1.778"/>
<pad name="DRDY" x="-6.35" y="-8.89" drill="0.8" diameter="1.778"/>
<circle x="5.08" y="7.62" radius="1.27" width="0.127" layer="21"/>
</package>
<package name="CAP">
<pad name="A" x="-3.81" y="0" drill="0.9" diameter="1.778"/>
<pad name="B" x="3.81" y="0" drill="0.9" diameter="1.778"/>
<wire x1="3.81" y1="0" x2="1.27" y2="0" width="0.127" layer="21"/>
<wire x1="1.27" y1="0" x2="1.27" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.27" y1="0" x2="1.27" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.127" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="0" x2="-3.81" y2="0" width="0.127" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="MOTOR_HEADER">
<wire x1="-7.62" y1="5.08" x2="-7.62" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-7.62" x2="10.16" y2="-7.62" width="0.254" layer="94"/>
<wire x1="10.16" y1="-7.62" x2="10.16" y2="5.08" width="0.254" layer="94"/>
<wire x1="10.16" y1="5.08" x2="-7.62" y2="5.08" width="0.254" layer="94"/>
<pin name="FL" x="-12.7" y="2.54" length="middle"/>
<pin name="FR" x="-12.7" y="0" length="middle"/>
<pin name="BL" x="-12.7" y="-2.54" length="middle"/>
<pin name="BR" x="-12.7" y="-5.08" length="middle"/>
<pin name="GND1" x="15.24" y="2.54" length="middle" rot="R180"/>
<pin name="GND2" x="15.24" y="0" length="middle" rot="R180"/>
<pin name="GND3" x="15.24" y="-2.54" length="middle" rot="R180"/>
<pin name="GND4" x="15.24" y="-5.08" length="middle" rot="R180"/>
</symbol>
<symbol name="SAMD21G-MINI">
<wire x1="-22.86" y1="33.02" x2="-22.86" y2="-33.02" width="0.254" layer="94"/>
<wire x1="-22.86" y1="-33.02" x2="22.86" y2="-33.02" width="0.254" layer="94"/>
<wire x1="22.86" y1="-33.02" x2="22.86" y2="33.02" width="0.254" layer="94"/>
<pin name="9" x="-27.94" y="-27.94" length="middle"/>
<pin name="8" x="-27.94" y="-22.86" length="middle"/>
<pin name="7" x="-27.94" y="-17.78" length="middle"/>
<pin name="6" x="-27.94" y="-12.7" length="middle"/>
<pin name="5" x="-27.94" y="-7.62" length="middle"/>
<pin name="4" x="-27.94" y="-2.54" length="middle"/>
<pin name="3" x="-27.94" y="2.54" length="middle"/>
<pin name="2" x="-27.94" y="7.62" length="middle"/>
<pin name="GND2" x="-27.94" y="12.7" length="middle"/>
<pin name="RSTN" x="-27.94" y="17.78" length="middle"/>
<pin name="0/RX" x="-27.94" y="22.86" length="middle"/>
<pin name="1/TX" x="-27.94" y="27.94" length="middle"/>
<pin name="10" x="27.94" y="-27.94" length="middle" rot="R180"/>
<pin name="11" x="27.94" y="-22.86" length="middle" rot="R180"/>
<pin name="12" x="27.94" y="-17.78" length="middle" rot="R180"/>
<pin name="13" x="27.94" y="-12.7" length="middle" rot="R180"/>
<pin name="A0" x="27.94" y="-7.62" length="middle" rot="R180"/>
<pin name="A1" x="27.94" y="-2.54" length="middle" rot="R180"/>
<pin name="A2" x="27.94" y="2.54" length="middle" rot="R180"/>
<pin name="A3" x="27.94" y="7.62" length="middle" rot="R180"/>
<pin name="VCC" x="27.94" y="12.7" length="middle" rot="R180"/>
<pin name="AREF" x="27.94" y="17.78" length="middle" rot="R180"/>
<pin name="GND" x="27.94" y="22.86" length="middle" rot="R180"/>
<pin name="VIN" x="27.94" y="27.94" length="middle" rot="R180"/>
<wire x1="-22.86" y1="33.02" x2="22.86" y2="33.02" width="0.254" layer="94"/>
<text x="-7.62" y="0" size="1.778" layer="94">SAMD21G_mini</text>
<pin name="SDA" x="2.54" y="-38.1" length="middle" rot="R90"/>
<pin name="SCL" x="-2.54" y="-38.1" length="middle" rot="R90"/>
</symbol>
<symbol name="LEVELSHIFT_8CH">
<wire x1="-12.7" y1="15.24" x2="-12.7" y2="-17.78" width="0.254" layer="94"/>
<wire x1="12.7" y1="-17.78" x2="12.7" y2="15.24" width="0.254" layer="94"/>
<pin name="VCCB" x="17.78" y="12.7" length="middle" rot="R180"/>
<pin name="B1" x="17.78" y="7.62" length="middle" rot="R180"/>
<pin name="B2" x="17.78" y="5.08" length="middle" rot="R180"/>
<pin name="B3" x="17.78" y="2.54" length="middle" rot="R180"/>
<pin name="B4" x="17.78" y="0" length="middle" rot="R180"/>
<pin name="B5" x="17.78" y="-2.54" length="middle" rot="R180"/>
<pin name="B6" x="17.78" y="-5.08" length="middle" rot="R180"/>
<pin name="B7" x="17.78" y="-7.62" length="middle" rot="R180"/>
<pin name="B8" x="17.78" y="-10.16" length="middle" rot="R180"/>
<pin name="VCCA" x="-17.78" y="12.7" length="middle"/>
<pin name="A1" x="-17.78" y="7.62" length="middle"/>
<pin name="A2" x="-17.78" y="5.08" length="middle"/>
<pin name="A3" x="-17.78" y="2.54" length="middle"/>
<pin name="A4" x="-17.78" y="0" length="middle"/>
<pin name="A5" x="-17.78" y="-2.54" length="middle"/>
<pin name="A6" x="-17.78" y="-5.08" length="middle"/>
<pin name="A7" x="-17.78" y="-7.62" length="middle"/>
<pin name="A8" x="-17.78" y="-10.16" length="middle"/>
<pin name="OE" x="-17.78" y="-15.24" length="middle"/>
<pin name="GND" x="17.78" y="-15.24" length="middle" rot="R180"/>
<wire x1="-12.7" y1="15.24" x2="12.7" y2="15.24" width="0.254" layer="94"/>
<wire x1="12.7" y1="-17.78" x2="-12.7" y2="-17.78" width="0.254" layer="94"/>
</symbol>
<symbol name="RADIO_8CH">
<wire x1="-5.08" y1="15.24" x2="5.08" y2="15.24" width="0.254" layer="94"/>
<wire x1="5.08" y1="15.24" x2="5.08" y2="-15.24" width="0.254" layer="94"/>
<wire x1="5.08" y1="-15.24" x2="-5.08" y2="-15.24" width="0.254" layer="94"/>
<wire x1="-5.08" y1="-15.24" x2="-5.08" y2="15.24" width="0.254" layer="94"/>
<pin name="CH1" x="-10.16" y="12.7" length="middle"/>
<pin name="CH2" x="-10.16" y="10.16" length="middle"/>
<pin name="CH3" x="-10.16" y="7.62" length="middle"/>
<pin name="CH4" x="-10.16" y="5.08" length="middle"/>
<pin name="CH5" x="-10.16" y="2.54" length="middle"/>
<pin name="CH6" x="-10.16" y="0" length="middle"/>
<pin name="CH7" x="-10.16" y="-2.54" length="middle"/>
<pin name="CH8" x="-10.16" y="-5.08" length="middle"/>
<pin name="VCC" x="-10.16" y="-10.16" length="middle"/>
<pin name="GND" x="-10.16" y="-12.7" length="middle"/>
</symbol>
<symbol name="LEVELSHIFT_4CH">
<wire x1="-10.16" y1="12.7" x2="-10.16" y2="-10.16" width="0.254" layer="94"/>
<wire x1="10.16" y1="-10.16" x2="10.16" y2="12.7" width="0.254" layer="94"/>
<pin name="HV" x="15.24" y="10.16" length="middle" rot="R180"/>
<pin name="B1" x="15.24" y="5.08" length="middle" rot="R180"/>
<pin name="B2" x="15.24" y="2.54" length="middle" rot="R180"/>
<pin name="B3" x="15.24" y="0" length="middle" rot="R180"/>
<pin name="B4" x="15.24" y="-2.54" length="middle" rot="R180"/>
<pin name="GND2" x="15.24" y="-7.62" length="middle" rot="R180"/>
<pin name="LV" x="-15.24" y="10.16" length="middle"/>
<pin name="A1" x="-15.24" y="5.08" length="middle"/>
<pin name="A2" x="-15.24" y="2.54" length="middle"/>
<pin name="A3" x="-15.24" y="0" length="middle"/>
<pin name="A4" x="-15.24" y="-2.54" length="middle"/>
<pin name="GND" x="-15.24" y="-7.62" length="middle"/>
<wire x1="-10.16" y1="12.7" x2="10.16" y2="12.7" width="0.254" layer="94"/>
<wire x1="10.16" y1="-10.16" x2="-10.16" y2="-10.16" width="0.254" layer="94"/>
</symbol>
<symbol name="GY-86">
<wire x1="-10.16" y1="12.7" x2="-10.16" y2="-12.7" width="0.254" layer="94"/>
<wire x1="-10.16" y1="-12.7" x2="10.16" y2="-12.7" width="0.254" layer="94"/>
<wire x1="10.16" y1="-12.7" x2="10.16" y2="12.7" width="0.254" layer="94"/>
<wire x1="10.16" y1="12.7" x2="-10.16" y2="12.7" width="0.254" layer="94"/>
<pin name="VCC" x="-15.24" y="10.16" length="middle"/>
<pin name="3V3" x="-15.24" y="7.62" length="middle"/>
<pin name="GND" x="-15.24" y="5.08" length="middle"/>
<pin name="SCL" x="-15.24" y="2.54" length="middle"/>
<pin name="SDA" x="-15.24" y="-2.54" length="middle"/>
<pin name="FSYNC" x="-15.24" y="-5.08" length="middle"/>
<pin name="INTA" x="-15.24" y="-7.62" length="middle"/>
<pin name="DRDY" x="-15.24" y="-10.16" length="middle"/>
</symbol>
<symbol name="CAP">
<wire x1="-2.54" y1="2.54" x2="-2.54" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="0" y2="2.54" width="0.254" layer="94"/>
<pin name="A" x="-7.62" y="0" length="middle"/>
<pin name="B" x="5.08" y="0" length="middle" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="MOTOR_HEADER">
<gates>
<gate name="G$1" symbol="MOTOR_HEADER" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MOTOR_HEADER">
<connects>
<connect gate="G$1" pin="BL" pad="BL"/>
<connect gate="G$1" pin="BR" pad="BR"/>
<connect gate="G$1" pin="FL" pad="FL"/>
<connect gate="G$1" pin="FR" pad="FR"/>
<connect gate="G$1" pin="GND1" pad="GND1"/>
<connect gate="G$1" pin="GND2" pad="GND2"/>
<connect gate="G$1" pin="GND3" pad="GND3"/>
<connect gate="G$1" pin="GND4" pad="GND4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="SAMD21G_MINI">
<gates>
<gate name="G$1" symbol="SAMD21G-MINI" x="101.6" y="-5.08"/>
</gates>
<devices>
<device name="" package="SAMD21G-MINI">
<connects>
<connect gate="G$1" pin="0/RX" pad="0/RX"/>
<connect gate="G$1" pin="1/TX" pad="1/RX"/>
<connect gate="G$1" pin="10" pad="10"/>
<connect gate="G$1" pin="11" pad="11"/>
<connect gate="G$1" pin="12" pad="12"/>
<connect gate="G$1" pin="13" pad="13"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
<connect gate="G$1" pin="A0" pad="A0"/>
<connect gate="G$1" pin="A1" pad="A1"/>
<connect gate="G$1" pin="A2" pad="A2"/>
<connect gate="G$1" pin="A3" pad="A3"/>
<connect gate="G$1" pin="AREF" pad="AREF"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="GND2" pad="GND2"/>
<connect gate="G$1" pin="RSTN" pad="RSTN"/>
<connect gate="G$1" pin="SCL" pad="SCL"/>
<connect gate="G$1" pin="SDA" pad="SDA"/>
<connect gate="G$1" pin="VCC" pad="VCC"/>
<connect gate="G$1" pin="VIN" pad="VIN"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="LEVELSHIFT_8CH">
<gates>
<gate name="G$1" symbol="LEVELSHIFT_8CH" x="5.08" y="5.08"/>
</gates>
<devices>
<device name="" package="LEVELSHIFT_8CH">
<connects>
<connect gate="G$1" pin="A1" pad="A1"/>
<connect gate="G$1" pin="A2" pad="A2"/>
<connect gate="G$1" pin="A3" pad="A3"/>
<connect gate="G$1" pin="A4" pad="A4"/>
<connect gate="G$1" pin="A5" pad="A5"/>
<connect gate="G$1" pin="A6" pad="A6"/>
<connect gate="G$1" pin="A7" pad="A7"/>
<connect gate="G$1" pin="A8" pad="A8"/>
<connect gate="G$1" pin="B1" pad="B1"/>
<connect gate="G$1" pin="B2" pad="B2"/>
<connect gate="G$1" pin="B3" pad="B3"/>
<connect gate="G$1" pin="B4" pad="B4"/>
<connect gate="G$1" pin="B5" pad="B5"/>
<connect gate="G$1" pin="B6" pad="B6"/>
<connect gate="G$1" pin="B7" pad="B7"/>
<connect gate="G$1" pin="B8" pad="B8"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="OE" pad="OE"/>
<connect gate="G$1" pin="VCCA" pad="VCCA"/>
<connect gate="G$1" pin="VCCB" pad="VCCB"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="RADIO_8CH">
<gates>
<gate name="G$1" symbol="RADIO_8CH" x="0" y="0"/>
</gates>
<devices>
<device name="" package="RADIO_8CH">
<connects>
<connect gate="G$1" pin="CH1" pad="CH1"/>
<connect gate="G$1" pin="CH2" pad="CH2"/>
<connect gate="G$1" pin="CH3" pad="CH3"/>
<connect gate="G$1" pin="CH4" pad="CH4"/>
<connect gate="G$1" pin="CH5" pad="CH5"/>
<connect gate="G$1" pin="CH6" pad="CH6"/>
<connect gate="G$1" pin="CH7" pad="CH7"/>
<connect gate="G$1" pin="CH8" pad="CH8"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="VCC" pad="VCC"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="LEVELSHIFT_4CH">
<gates>
<gate name="G$1" symbol="LEVELSHIFT_4CH" x="0" y="0"/>
</gates>
<devices>
<device name="" package="LEVELSHIFT_4CH">
<connects>
<connect gate="G$1" pin="A1" pad="A1"/>
<connect gate="G$1" pin="A2" pad="A2"/>
<connect gate="G$1" pin="A3" pad="A3"/>
<connect gate="G$1" pin="A4" pad="A4"/>
<connect gate="G$1" pin="B1" pad="B1"/>
<connect gate="G$1" pin="B2" pad="B2"/>
<connect gate="G$1" pin="B3" pad="B3"/>
<connect gate="G$1" pin="B4" pad="B4"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="GND2" pad="GND2"/>
<connect gate="G$1" pin="HV" pad="HV"/>
<connect gate="G$1" pin="LV" pad="LV"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="GY-86">
<description>10 DoF breakout board. Combined MPU6050, HMC5883L and MS5611</description>
<gates>
<gate name="G$1" symbol="GY-86" x="0" y="0"/>
</gates>
<devices>
<device name="" package="GY-86">
<connects>
<connect gate="G$1" pin="3V3" pad="3V3"/>
<connect gate="G$1" pin="DRDY" pad="DRDY"/>
<connect gate="G$1" pin="FSYNC" pad="FSYNC"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="INTA" pad="INTA"/>
<connect gate="G$1" pin="SCL" pad="SCL"/>
<connect gate="G$1" pin="SDA" pad="SDA"/>
<connect gate="G$1" pin="VCC" pad="VCC"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CAP">
<gates>
<gate name="G$1" symbol="CAP" x="2.54" y="0"/>
</gates>
<devices>
<device name="" package="CAP">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="B" pad="B"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-lstb">
<description>&lt;b&gt;Pin Headers&lt;/b&gt;&lt;p&gt;
Naming:&lt;p&gt;
MA = male&lt;p&gt;
# contacts - # rows&lt;p&gt;
W = angled&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="MA03-2">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-3.175" y1="2.54" x2="-1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="2.54" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.905" x2="-0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.54" x2="1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="2.54" x2="3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.54" x2="3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.905" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-2.54" x2="-1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.905" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.905" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-2.54" x2="1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="-1.27" drill="1.016"/>
<pad name="3" x="0" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="5" x="2.54" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="2" x="-2.54" y="1.27" drill="1.016" shape="octagon"/>
<pad name="4" x="0" y="1.27" drill="1.016" shape="octagon"/>
<pad name="6" x="2.54" y="1.27" drill="1.016" shape="octagon"/>
<text x="-3.175" y="-4.191" size="1.27" layer="21" ratio="10">1</text>
<text x="-3.81" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="4.064" y="0.635" size="1.27" layer="21" ratio="10">6</text>
<text x="-1.27" y="-4.191" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-1.016" layer="51"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="1.016" x2="-2.286" y2="1.524" layer="51"/>
<rectangle x1="-0.254" y1="1.016" x2="0.254" y2="1.524" layer="51"/>
<rectangle x1="2.286" y1="1.016" x2="2.794" y2="1.524" layer="51"/>
</package>
<package name="MA04-1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-4.445" y1="1.27" x2="-3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.635" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-1.27" x2="-2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.635" x2="-5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-1.27" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="4.445" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="1.27" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0.635" x2="5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-0.635" x2="4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="-1.27" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-5.08" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.223" y="-0.635" size="1.27" layer="21" ratio="10">1</text>
<text x="0.635" y="1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="5.334" y="-0.635" size="1.27" layer="21" ratio="10">4</text>
<rectangle x1="-1.524" y1="-0.254" x2="-1.016" y2="0.254" layer="51"/>
<rectangle x1="-4.064" y1="-0.254" x2="-3.556" y2="0.254" layer="51"/>
<rectangle x1="1.016" y1="-0.254" x2="1.524" y2="0.254" layer="51"/>
<rectangle x1="3.556" y1="-0.254" x2="4.064" y2="0.254" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="MA03-2">
<wire x1="3.81" y1="-5.08" x2="-3.81" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="5.08" x2="-3.81" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-1.27" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-2.54" x2="-1.27" y2="-2.54" width="0.6096" layer="94"/>
<text x="-3.81" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<text x="-3.81" y="5.842" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="-7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="4" x="-7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="6" x="-7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1"/>
</symbol>
<symbol name="MA04-1">
<wire x1="3.81" y1="-7.62" x2="-1.27" y2="-7.62" width="0.4064" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-5.08" x2="2.54" y2="-5.08" width="0.6096" layer="94"/>
<wire x1="-1.27" y1="5.08" x2="-1.27" y2="-7.62" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-7.62" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-1.27" y1="5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<text x="-1.27" y="-10.16" size="1.778" layer="96">&gt;VALUE</text>
<text x="-1.27" y="5.842" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="4" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="MA03-2" prefix="SV" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="MA03-2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA03-2">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="1" pin="3" pad="3"/>
<connect gate="1" pin="4" pad="4"/>
<connect gate="1" pin="5" pad="5"/>
<connect gate="1" pin="6" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MA04-1" prefix="SV" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="MA04-1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA04-1">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="1" pin="3" pad="3"/>
<connect gate="1" pin="4" pad="4"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U$1" library="quadcopter_comp" deviceset="MOTOR_HEADER" device=""/>
<part name="U$3" library="quadcopter_comp" deviceset="SAMD21G_MINI" device=""/>
<part name="U$4" library="quadcopter_comp" deviceset="LEVELSHIFT_8CH" device=""/>
<part name="U$6" library="quadcopter_comp" deviceset="RADIO_8CH" device=""/>
<part name="I2C" library="con-lstb" deviceset="MA03-2" device=""/>
<part name="3V" library="con-lstb" deviceset="MA03-2" device=""/>
<part name="5V" library="con-lstb" deviceset="MA03-2" device=""/>
<part name="SERIAL" library="con-lstb" deviceset="MA04-1" device=""/>
<part name="U$5" library="quadcopter_comp" deviceset="LEVELSHIFT_4CH" device=""/>
<part name="U$8" library="quadcopter_comp" deviceset="GY-86" device=""/>
<part name="U$2" library="quadcopter_comp" deviceset="CAP" device=""/>
<part name="U$7" library="quadcopter_comp" deviceset="CAP" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="101.6" y="0"/>
<instance part="U$3" gate="G$1" x="0" y="0"/>
<instance part="U$4" gate="G$1" x="-30.48" y="71.12"/>
<instance part="U$6" gate="G$1" x="7.62" y="66.04"/>
<instance part="I2C" gate="1" x="-22.86" y="-53.34" rot="R90"/>
<instance part="3V" gate="1" x="60.96" y="-33.02" rot="R90"/>
<instance part="5V" gate="1" x="109.22" y="33.02" rot="R90"/>
<instance part="SERIAL" gate="1" x="-53.34" y="38.1"/>
<instance part="U$5" gate="G$1" x="68.58" y="-2.54"/>
<instance part="U$8" gate="G$1" x="91.44" y="-58.42"/>
<instance part="U$2" gate="G$1" x="96.52" y="35.56" rot="R90"/>
<instance part="U$7" gate="G$1" x="121.92" y="35.56" rot="R90"/>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="BR"/>
<wire x1="83.82" y1="-5.08" x2="88.9" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="B4"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="BL"/>
<wire x1="88.9" y1="-2.54" x2="83.82" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="B3"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="FR"/>
<wire x1="83.82" y1="0" x2="88.9" y2="0" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="B2"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="FL"/>
<wire x1="88.9" y1="2.54" x2="83.82" y2="2.54" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="B1"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="10"/>
<wire x1="27.94" y1="-27.94" x2="45.72" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="45.72" y1="-27.94" x2="45.72" y2="2.54" width="0.1524" layer="91"/>
<wire x1="45.72" y1="2.54" x2="53.34" y2="2.54" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="A1"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<wire x1="53.34" y1="0" x2="43.18" y2="0" width="0.1524" layer="91"/>
<wire x1="43.18" y1="0" x2="43.18" y2="-22.86" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="11"/>
<wire x1="43.18" y1="-22.86" x2="27.94" y2="-22.86" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="A2"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="12"/>
<wire x1="27.94" y1="-17.78" x2="40.64" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="40.64" y1="-17.78" x2="40.64" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="40.64" y1="-2.54" x2="53.34" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="A3"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<wire x1="53.34" y1="-5.08" x2="38.1" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="38.1" y1="-5.08" x2="38.1" y2="-12.7" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="13"/>
<wire x1="38.1" y1="-12.7" x2="27.94" y2="-12.7" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="A4"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="U$4" gate="G$1" pin="B1"/>
<pinref part="U$6" gate="G$1" pin="CH1"/>
<wire x1="-12.7" y1="78.74" x2="-2.54" y2="78.74" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="U$6" gate="G$1" pin="CH2"/>
<pinref part="U$4" gate="G$1" pin="B2"/>
<wire x1="-2.54" y1="76.2" x2="-12.7" y2="76.2" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="U$4" gate="G$1" pin="B3"/>
<pinref part="U$6" gate="G$1" pin="CH3"/>
<wire x1="-12.7" y1="73.66" x2="-2.54" y2="73.66" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="U$6" gate="G$1" pin="CH4"/>
<pinref part="U$4" gate="G$1" pin="B4"/>
<wire x1="-2.54" y1="71.12" x2="-12.7" y2="71.12" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="U$4" gate="G$1" pin="B5"/>
<pinref part="U$6" gate="G$1" pin="CH5"/>
<wire x1="-12.7" y1="68.58" x2="-2.54" y2="68.58" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="U$6" gate="G$1" pin="CH6"/>
<pinref part="U$4" gate="G$1" pin="B6"/>
<wire x1="-2.54" y1="66.04" x2="-12.7" y2="66.04" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="U$4" gate="G$1" pin="B7"/>
<pinref part="U$6" gate="G$1" pin="CH7"/>
<wire x1="-12.7" y1="63.5" x2="-2.54" y2="63.5" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<pinref part="U$6" gate="G$1" pin="CH8"/>
<pinref part="U$4" gate="G$1" pin="B8"/>
<wire x1="-2.54" y1="60.96" x2="-12.7" y2="60.96" width="0.1524" layer="91"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="U$6" gate="G$1" pin="GND"/>
<wire x1="-2.54" y1="53.34" x2="-10.16" y2="53.34" width="0.1524" layer="91"/>
<wire x1="-10.16" y1="53.34" x2="-10.16" y2="55.88" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="GND"/>
<wire x1="-10.16" y1="55.88" x2="-12.7" y2="55.88" width="0.1524" layer="91"/>
<pinref part="SERIAL" gate="1" pin="3"/>
<wire x1="-45.72" y1="38.1" x2="-33.02" y2="38.1" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="38.1" x2="-10.16" y2="38.1" width="0.1524" layer="91"/>
<wire x1="-10.16" y1="38.1" x2="48.26" y2="38.1" width="0.1524" layer="91"/>
<wire x1="48.26" y1="38.1" x2="48.26" y2="22.86" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="GND"/>
<wire x1="48.26" y1="22.86" x2="27.94" y2="22.86" width="0.1524" layer="91"/>
<wire x1="83.82" y1="-10.16" x2="86.36" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="86.36" y1="-10.16" x2="86.36" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="86.36" y1="-17.78" x2="63.5" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="63.5" y1="-17.78" x2="60.96" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="60.96" y1="-17.78" x2="58.42" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="58.42" y1="-17.78" x2="50.8" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="50.8" y1="-17.78" x2="50.8" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="50.8" y1="-10.16" x2="53.34" y2="-10.16" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="GND1"/>
<wire x1="116.84" y1="2.54" x2="121.92" y2="2.54" width="0.1524" layer="91"/>
<wire x1="121.92" y1="2.54" x2="121.92" y2="0" width="0.1524" layer="91"/>
<wire x1="121.92" y1="0" x2="121.92" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="121.92" y1="-2.54" x2="121.92" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="121.92" y1="-5.08" x2="121.92" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="121.92" y1="-17.78" x2="86.36" y2="-17.78" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="GND2"/>
<wire x1="116.84" y1="0" x2="121.92" y2="0" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="GND3"/>
<wire x1="116.84" y1="-2.54" x2="121.92" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="GND4"/>
<wire x1="116.84" y1="-5.08" x2="121.92" y2="-5.08" width="0.1524" layer="91"/>
<junction x="86.36" y="-17.78"/>
<junction x="121.92" y="0"/>
<junction x="121.92" y="-2.54"/>
<junction x="121.92" y="-5.08"/>
<wire x1="76.2" y1="-53.34" x2="50.8" y2="-53.34" width="0.1524" layer="91"/>
<wire x1="50.8" y1="-53.34" x2="50.8" y2="-17.78" width="0.1524" layer="91"/>
<junction x="50.8" y="-17.78"/>
<pinref part="5V" gate="1" pin="6"/>
<wire x1="106.68" y1="25.4" x2="106.68" y2="22.86" width="0.1524" layer="91"/>
<wire x1="106.68" y1="22.86" x2="109.22" y2="22.86" width="0.1524" layer="91"/>
<wire x1="109.22" y1="22.86" x2="111.76" y2="22.86" width="0.1524" layer="91"/>
<wire x1="111.76" y1="22.86" x2="121.92" y2="22.86" width="0.1524" layer="91"/>
<wire x1="121.92" y1="22.86" x2="121.92" y2="2.54" width="0.1524" layer="91"/>
<pinref part="5V" gate="1" pin="4"/>
<wire x1="109.22" y1="25.4" x2="109.22" y2="22.86" width="0.1524" layer="91"/>
<pinref part="5V" gate="1" pin="2"/>
<wire x1="111.76" y1="25.4" x2="111.76" y2="22.86" width="0.1524" layer="91"/>
<junction x="109.22" y="22.86"/>
<junction x="111.76" y="22.86"/>
<junction x="121.92" y="2.54"/>
<pinref part="3V" gate="1" pin="1"/>
<wire x1="63.5" y1="-25.4" x2="63.5" y2="-17.78" width="0.1524" layer="91"/>
<pinref part="3V" gate="1" pin="3"/>
<wire x1="60.96" y1="-25.4" x2="60.96" y2="-17.78" width="0.1524" layer="91"/>
<pinref part="3V" gate="1" pin="5"/>
<wire x1="58.42" y1="-25.4" x2="58.42" y2="-17.78" width="0.1524" layer="91"/>
<junction x="63.5" y="-17.78"/>
<junction x="60.96" y="-17.78"/>
<junction x="58.42" y="-17.78"/>
<wire x1="48.26" y1="22.86" x2="96.52" y2="22.86" width="0.1524" layer="91"/>
<wire x1="96.52" y1="22.86" x2="106.68" y2="22.86" width="0.1524" layer="91"/>
<wire x1="-10.16" y1="53.34" x2="-10.16" y2="38.1" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="GND2"/>
<wire x1="-27.94" y1="12.7" x2="-33.02" y2="12.7" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="12.7" x2="-33.02" y2="38.1" width="0.1524" layer="91"/>
<junction x="-33.02" y="38.1"/>
<junction x="-10.16" y="38.1"/>
<junction x="-10.16" y="53.34"/>
<junction x="48.26" y="22.86"/>
<junction x="106.68" y="22.86"/>
<pinref part="U$5" gate="G$1" pin="GND2"/>
<pinref part="U$5" gate="G$1" pin="GND"/>
<pinref part="U$8" gate="G$1" pin="GND"/>
<pinref part="U$2" gate="G$1" pin="A"/>
<wire x1="96.52" y1="27.94" x2="96.52" y2="22.86" width="0.1524" layer="91"/>
<pinref part="U$7" gate="G$1" pin="A"/>
<wire x1="121.92" y1="27.94" x2="121.92" y2="22.86" width="0.1524" layer="91"/>
<junction x="96.52" y="22.86"/>
<junction x="121.92" y="22.86"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="9"/>
<wire x1="-27.94" y1="-27.94" x2="-76.2" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="-76.2" y1="-27.94" x2="-76.2" y2="78.74" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="A1"/>
<wire x1="-76.2" y1="78.74" x2="-48.26" y2="78.74" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="U$4" gate="G$1" pin="A2"/>
<wire x1="-48.26" y1="76.2" x2="-73.66" y2="76.2" width="0.1524" layer="91"/>
<wire x1="-73.66" y1="76.2" x2="-73.66" y2="-22.86" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="8"/>
<wire x1="-27.94" y1="-22.86" x2="-73.66" y2="-22.86" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="7"/>
<wire x1="-27.94" y1="-17.78" x2="-71.12" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="-71.12" y1="-17.78" x2="-71.12" y2="73.66" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="A3"/>
<wire x1="-71.12" y1="73.66" x2="-48.26" y2="73.66" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$22" class="0">
<segment>
<pinref part="U$4" gate="G$1" pin="A4"/>
<wire x1="-48.26" y1="71.12" x2="-68.58" y2="71.12" width="0.1524" layer="91"/>
<wire x1="-68.58" y1="71.12" x2="-68.58" y2="-12.7" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="6"/>
<wire x1="-68.58" y1="-12.7" x2="-27.94" y2="-12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="5"/>
<wire x1="-27.94" y1="-7.62" x2="-66.04" y2="-7.62" width="0.1524" layer="91"/>
<wire x1="-66.04" y1="-7.62" x2="-66.04" y2="68.58" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="A5"/>
<wire x1="-66.04" y1="68.58" x2="-48.26" y2="68.58" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="U$4" gate="G$1" pin="A6"/>
<wire x1="-48.26" y1="66.04" x2="-63.5" y2="66.04" width="0.1524" layer="91"/>
<wire x1="-63.5" y1="66.04" x2="-63.5" y2="2.54" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="3"/>
<wire x1="-27.94" y1="2.54" x2="-63.5" y2="2.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="5V" class="0">
<segment>
<wire x1="83.82" y1="7.62" x2="88.9" y2="7.62" width="0.1524" layer="91"/>
<wire x1="88.9" y1="7.62" x2="88.9" y2="27.94" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="VCCB"/>
<wire x1="88.9" y1="27.94" x2="88.9" y2="45.72" width="0.1524" layer="91"/>
<wire x1="88.9" y1="45.72" x2="88.9" y2="48.26" width="0.1524" layer="91"/>
<wire x1="88.9" y1="48.26" x2="88.9" y2="83.82" width="0.1524" layer="91"/>
<wire x1="-12.7" y1="83.82" x2="88.9" y2="83.82" width="0.1524" layer="91"/>
<pinref part="U$6" gate="G$1" pin="VCC"/>
<wire x1="-2.54" y1="55.88" x2="-7.62" y2="55.88" width="0.1524" layer="91"/>
<wire x1="-7.62" y1="55.88" x2="-7.62" y2="48.26" width="0.1524" layer="91"/>
<wire x1="-7.62" y1="48.26" x2="88.9" y2="48.26" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="VIN"/>
<wire x1="27.94" y1="27.94" x2="88.9" y2="27.94" width="0.1524" layer="91"/>
<junction x="88.9" y="27.94"/>
<junction x="88.9" y="48.26"/>
<pinref part="5V" gate="1" pin="1"/>
<wire x1="111.76" y1="40.64" x2="111.76" y2="45.72" width="0.1524" layer="91"/>
<wire x1="111.76" y1="45.72" x2="109.22" y2="45.72" width="0.1524" layer="91"/>
<pinref part="5V" gate="1" pin="3"/>
<wire x1="109.22" y1="45.72" x2="106.68" y2="45.72" width="0.1524" layer="91"/>
<wire x1="106.68" y1="45.72" x2="96.52" y2="45.72" width="0.1524" layer="91"/>
<wire x1="96.52" y1="45.72" x2="88.9" y2="45.72" width="0.1524" layer="91"/>
<wire x1="109.22" y1="40.64" x2="109.22" y2="45.72" width="0.1524" layer="91"/>
<pinref part="5V" gate="1" pin="5"/>
<wire x1="106.68" y1="40.64" x2="106.68" y2="45.72" width="0.1524" layer="91"/>
<junction x="88.9" y="45.72"/>
<junction x="106.68" y="45.72"/>
<junction x="109.22" y="45.72"/>
<pinref part="U$5" gate="G$1" pin="HV"/>
<pinref part="U$2" gate="G$1" pin="B"/>
<wire x1="96.52" y1="40.64" x2="96.52" y2="45.72" width="0.1524" layer="91"/>
<pinref part="U$7" gate="G$1" pin="B"/>
<wire x1="111.76" y1="45.72" x2="121.92" y2="45.72" width="0.1524" layer="91"/>
<wire x1="121.92" y1="45.72" x2="121.92" y2="40.64" width="0.1524" layer="91"/>
<junction x="96.52" y="45.72"/>
<junction x="111.76" y="45.72"/>
</segment>
</net>
<net name="3V" class="0">
<segment>
<wire x1="53.34" y1="7.62" x2="35.56" y2="7.62" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="VCC"/>
<wire x1="35.56" y1="12.7" x2="27.94" y2="12.7" width="0.1524" layer="91"/>
<wire x1="35.56" y1="12.7" x2="35.56" y2="40.64" width="0.1524" layer="91"/>
<wire x1="35.56" y1="40.64" x2="35.56" y2="88.9" width="0.1524" layer="91"/>
<wire x1="35.56" y1="88.9" x2="-53.34" y2="88.9" width="0.1524" layer="91"/>
<wire x1="-53.34" y1="88.9" x2="-53.34" y2="83.82" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="VCCA"/>
<wire x1="-53.34" y1="83.82" x2="-48.26" y2="83.82" width="0.1524" layer="91"/>
<junction x="35.56" y="12.7"/>
<wire x1="35.56" y1="12.7" x2="35.56" y2="7.62" width="0.1524" layer="91"/>
<wire x1="35.56" y1="7.62" x2="35.56" y2="-50.8" width="0.1524" layer="91"/>
<wire x1="35.56" y1="-50.8" x2="58.42" y2="-50.8" width="0.1524" layer="91"/>
<junction x="35.56" y="7.62"/>
<pinref part="3V" gate="1" pin="6"/>
<wire x1="58.42" y1="-50.8" x2="60.96" y2="-50.8" width="0.1524" layer="91"/>
<wire x1="60.96" y1="-50.8" x2="63.5" y2="-50.8" width="0.1524" layer="91"/>
<wire x1="63.5" y1="-50.8" x2="76.2" y2="-50.8" width="0.1524" layer="91"/>
<wire x1="58.42" y1="-40.64" x2="58.42" y2="-50.8" width="0.1524" layer="91"/>
<pinref part="3V" gate="1" pin="2"/>
<wire x1="63.5" y1="-40.64" x2="63.5" y2="-50.8" width="0.1524" layer="91"/>
<pinref part="3V" gate="1" pin="4"/>
<wire x1="60.96" y1="-40.64" x2="60.96" y2="-50.8" width="0.1524" layer="91"/>
<junction x="63.5" y="-50.8"/>
<junction x="60.96" y="-50.8"/>
<junction x="58.42" y="-50.8"/>
<pinref part="SERIAL" gate="1" pin="4"/>
<wire x1="-45.72" y1="40.64" x2="35.56" y2="40.64" width="0.1524" layer="91"/>
<junction x="35.56" y="40.64"/>
<pinref part="U$5" gate="G$1" pin="LV"/>
<pinref part="U$8" gate="G$1" pin="3V3"/>
</segment>
</net>
<net name="N$26" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="SCL"/>
<wire x1="76.2" y1="-55.88" x2="-2.54" y2="-55.88" width="0.1524" layer="91"/>
<wire x1="-2.54" y1="-55.88" x2="-2.54" y2="-40.64" width="0.1524" layer="91"/>
<pinref part="I2C" gate="1" pin="5"/>
<wire x1="-2.54" y1="-40.64" x2="-2.54" y2="-38.1" width="0.1524" layer="91"/>
<wire x1="-25.4" y1="-45.72" x2="-25.4" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="-25.4" y1="-40.64" x2="-22.86" y2="-40.64" width="0.1524" layer="91"/>
<pinref part="I2C" gate="1" pin="3"/>
<wire x1="-22.86" y1="-40.64" x2="-20.32" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="-40.64" x2="-2.54" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="-22.86" y1="-45.72" x2="-22.86" y2="-40.64" width="0.1524" layer="91"/>
<pinref part="I2C" gate="1" pin="1"/>
<wire x1="-20.32" y1="-45.72" x2="-20.32" y2="-40.64" width="0.1524" layer="91"/>
<junction x="-2.54" y="-40.64"/>
<junction x="-20.32" y="-40.64"/>
<junction x="-22.86" y="-40.64"/>
<pinref part="U$8" gate="G$1" pin="SCL"/>
</segment>
</net>
<net name="N$28" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="SDA"/>
<wire x1="2.54" y1="-66.04" x2="2.54" y2="-60.96" width="0.1524" layer="91"/>
<pinref part="I2C" gate="1" pin="6"/>
<wire x1="2.54" y1="-60.96" x2="2.54" y2="-38.1" width="0.1524" layer="91"/>
<wire x1="-25.4" y1="-60.96" x2="-25.4" y2="-66.04" width="0.1524" layer="91"/>
<wire x1="-25.4" y1="-66.04" x2="-22.86" y2="-66.04" width="0.1524" layer="91"/>
<wire x1="-22.86" y1="-66.04" x2="-20.32" y2="-66.04" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="-66.04" x2="2.54" y2="-66.04" width="0.1524" layer="91"/>
<pinref part="I2C" gate="1" pin="4"/>
<wire x1="-22.86" y1="-60.96" x2="-22.86" y2="-66.04" width="0.1524" layer="91"/>
<pinref part="I2C" gate="1" pin="2"/>
<wire x1="-20.32" y1="-60.96" x2="-20.32" y2="-66.04" width="0.1524" layer="91"/>
<junction x="-22.86" y="-66.04"/>
<junction x="-20.32" y="-66.04"/>
<pinref part="U$8" gate="G$1" pin="SDA"/>
<wire x1="76.2" y1="-60.96" x2="2.54" y2="-60.96" width="0.1524" layer="91"/>
<junction x="2.54" y="-60.96"/>
</segment>
</net>
<net name="N$33" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="0/RX"/>
<wire x1="-27.94" y1="22.86" x2="-43.18" y2="22.86" width="0.1524" layer="91"/>
<wire x1="-43.18" y1="22.86" x2="-43.18" y2="33.02" width="0.1524" layer="91"/>
<pinref part="SERIAL" gate="1" pin="1"/>
<wire x1="-43.18" y1="33.02" x2="-45.72" y2="33.02" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$34" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="1/TX"/>
<wire x1="-27.94" y1="27.94" x2="-40.64" y2="27.94" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="27.94" x2="-40.64" y2="35.56" width="0.1524" layer="91"/>
<pinref part="SERIAL" gate="1" pin="2"/>
<wire x1="-40.64" y1="35.56" x2="-45.72" y2="35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="2"/>
<wire x1="-27.94" y1="7.62" x2="-60.96" y2="7.62" width="0.1524" layer="91"/>
<wire x1="-60.96" y1="7.62" x2="-60.96" y2="63.5" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="A7"/>
<wire x1="-60.96" y1="63.5" x2="-48.26" y2="63.5" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="A0"/>
<wire x1="27.94" y1="-7.62" x2="30.48" y2="-7.62" width="0.1524" layer="91"/>
<wire x1="33.02" y1="-7.62" x2="30.48" y2="-7.62" width="0.1524" layer="91"/>
<wire x1="30.48" y1="-7.62" x2="30.48" y2="45.72" width="0.1524" layer="91"/>
<wire x1="30.48" y1="45.72" x2="-58.42" y2="45.72" width="0.1524" layer="91"/>
<wire x1="-58.42" y1="45.72" x2="-58.42" y2="60.96" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="A8"/>
<wire x1="-58.42" y1="60.96" x2="-48.26" y2="60.96" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
