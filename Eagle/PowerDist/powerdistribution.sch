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
<library name="solpad">
<description>&lt;b&gt;Solder Pads/Test Points&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="SE11">
<description>&lt;b&gt;SOLDER PAD&lt;/b&gt;&lt;p&gt;
drill 1.1 mm</description>
<wire x1="-1.397" y1="0.508" x2="-1.397" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.508" y1="-1.397" x2="1.397" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.508" y1="1.397" x2="1.397" y2="0.508" width="0.1524" layer="21"/>
<wire x1="1.397" y1="0.508" x2="1.397" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="-1.397" y1="0.508" x2="-0.508" y2="1.397" width="0.1524" layer="21"/>
<wire x1="-0.508" y1="1.397" x2="0.508" y2="1.397" width="0.1524" layer="21"/>
<wire x1="-0.508" y1="-1.397" x2="-1.397" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.508" y1="-1.397" x2="-0.508" y2="-1.397" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.1524" layer="51"/>
<circle x="0" y="0" radius="0.381" width="0.254" layer="51"/>
<pad name="MP" x="0" y="0" drill="1.1176" diameter="2.159" shape="octagon"/>
<text x="-1.27" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="0.381" size="0.0254" layer="27">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="LSP">
<wire x1="-1.016" y1="2.032" x2="1.016" y2="0" width="0.254" layer="94"/>
<wire x1="-1.016" y1="0" x2="1.016" y2="2.032" width="0.254" layer="94"/>
<circle x="0" y="1.016" radius="1.016" width="0.4064" layer="94"/>
<text x="-1.27" y="2.921" size="1.778" layer="95">&gt;NAME</text>
<pin name="MP" x="0" y="-2.54" visible="off" length="short" direction="pas" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="SE11" prefix="LSP">
<description>&lt;b&gt;SOLDER PAD&lt;/b&gt;&lt;p&gt; RTM, drill 1,1mm, distributor Buerklin, 07F810</description>
<gates>
<gate name="1" symbol="LSP" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SE11">
<connects>
<connect gate="1" pin="MP" pad="MP"/>
</connects>
<technologies>
<technology name=""/>
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
<part name="LSP1" library="solpad" deviceset="SE11" device=""/>
<part name="LSP2" library="solpad" deviceset="SE11" device=""/>
<part name="LSP3" library="solpad" deviceset="SE11" device=""/>
<part name="LSP4" library="solpad" deviceset="SE11" device=""/>
<part name="LSP5" library="solpad" deviceset="SE11" device=""/>
<part name="LSP6" library="solpad" deviceset="SE11" device=""/>
<part name="LSP7" library="solpad" deviceset="SE11" device=""/>
<part name="LSP8" library="solpad" deviceset="SE11" device=""/>
<part name="LSP9" library="solpad" deviceset="SE11" device=""/>
<part name="LSP10" library="solpad" deviceset="SE11" device=""/>
<part name="LSP11" library="solpad" deviceset="SE11" device=""/>
<part name="LSP12" library="solpad" deviceset="SE11" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="LSP1" gate="1" x="0" y="86.36" rot="R90"/>
<instance part="LSP2" gate="1" x="0" y="81.28" rot="R90"/>
<instance part="LSP3" gate="1" x="0" y="55.88" rot="R90"/>
<instance part="LSP4" gate="1" x="0" y="50.8" rot="R90"/>
<instance part="LSP5" gate="1" x="2.54" y="20.32" rot="R90"/>
<instance part="LSP6" gate="1" x="2.54" y="15.24" rot="R90"/>
<instance part="LSP7" gate="1" x="114.3" y="86.36" rot="R270"/>
<instance part="LSP8" gate="1" x="114.3" y="81.28" rot="R270"/>
<instance part="LSP9" gate="1" x="111.76" y="55.88" rot="R270"/>
<instance part="LSP10" gate="1" x="111.76" y="50.8" rot="R270"/>
<instance part="LSP11" gate="1" x="111.76" y="22.86" rot="R270"/>
<instance part="LSP12" gate="1" x="111.76" y="17.78" rot="R270"/>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<wire x1="2.54" y1="86.36" x2="40.64" y2="86.36" width="0.1524" layer="91"/>
<wire x1="40.64" y1="86.36" x2="40.64" y2="81.28" width="0.1524" layer="91"/>
<wire x1="40.64" y1="81.28" x2="40.64" y2="55.88" width="0.1524" layer="91"/>
<wire x1="40.64" y1="55.88" x2="40.64" y2="50.8" width="0.1524" layer="91"/>
<wire x1="40.64" y1="50.8" x2="40.64" y2="20.32" width="0.1524" layer="91"/>
<wire x1="40.64" y1="20.32" x2="40.64" y2="17.78" width="0.1524" layer="91"/>
<wire x1="111.76" y1="81.28" x2="40.64" y2="81.28" width="0.1524" layer="91"/>
<wire x1="109.22" y1="50.8" x2="40.64" y2="50.8" width="0.1524" layer="91"/>
<wire x1="109.22" y1="17.78" x2="40.64" y2="17.78" width="0.1524" layer="91"/>
<wire x1="2.54" y1="55.88" x2="40.64" y2="55.88" width="0.1524" layer="91"/>
<wire x1="5.08" y1="20.32" x2="40.64" y2="20.32" width="0.1524" layer="91"/>
<junction x="40.64" y="81.28"/>
<junction x="40.64" y="55.88"/>
<junction x="40.64" y="20.32"/>
<junction x="40.64" y="50.8"/>
<pinref part="LSP1" gate="1" pin="MP"/>
<pinref part="LSP3" gate="1" pin="MP"/>
<pinref part="LSP5" gate="1" pin="MP"/>
<pinref part="LSP8" gate="1" pin="MP"/>
<pinref part="LSP10" gate="1" pin="MP"/>
<pinref part="LSP12" gate="1" pin="MP"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<wire x1="5.08" y1="15.24" x2="45.72" y2="15.24" width="0.1524" layer="91"/>
<wire x1="45.72" y1="15.24" x2="45.72" y2="22.86" width="0.1524" layer="91"/>
<wire x1="45.72" y1="22.86" x2="45.72" y2="45.72" width="0.1524" layer="91"/>
<wire x1="45.72" y1="45.72" x2="45.72" y2="55.88" width="0.1524" layer="91"/>
<wire x1="45.72" y1="55.88" x2="45.72" y2="76.2" width="0.1524" layer="91"/>
<wire x1="45.72" y1="76.2" x2="45.72" y2="86.36" width="0.1524" layer="91"/>
<wire x1="45.72" y1="86.36" x2="111.76" y2="86.36" width="0.1524" layer="91"/>
<wire x1="109.22" y1="55.88" x2="45.72" y2="55.88" width="0.1524" layer="91"/>
<wire x1="109.22" y1="22.86" x2="45.72" y2="22.86" width="0.1524" layer="91"/>
<wire x1="2.54" y1="50.8" x2="33.02" y2="50.8" width="0.1524" layer="91"/>
<wire x1="33.02" y1="50.8" x2="33.02" y2="45.72" width="0.1524" layer="91"/>
<wire x1="33.02" y1="45.72" x2="45.72" y2="45.72" width="0.1524" layer="91"/>
<wire x1="2.54" y1="81.28" x2="33.02" y2="81.28" width="0.1524" layer="91"/>
<wire x1="33.02" y1="81.28" x2="33.02" y2="76.2" width="0.1524" layer="91"/>
<wire x1="33.02" y1="76.2" x2="45.72" y2="76.2" width="0.1524" layer="91"/>
<junction x="45.72" y="76.2"/>
<junction x="45.72" y="55.88"/>
<junction x="45.72" y="22.86"/>
<junction x="45.72" y="45.72"/>
<pinref part="LSP2" gate="1" pin="MP"/>
<pinref part="LSP4" gate="1" pin="MP"/>
<pinref part="LSP6" gate="1" pin="MP"/>
<pinref part="LSP7" gate="1" pin="MP"/>
<pinref part="LSP9" gate="1" pin="MP"/>
<pinref part="LSP11" gate="1" pin="MP"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
