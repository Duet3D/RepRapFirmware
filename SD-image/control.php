<!DOCTYPE HTML>
<head>
<style type="text/css">td { text-align: center; } </style>
</head>

<html>

<h2>RepRap: 
<?php print(getMyName()); ?>
<?php if(gotPassword()) echo '&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<a href="http://reprappro.com" target="_blank"><img src="logo.png" alt="RepRapPro logo"></a>'; ?>
</h2><br><br>
<?php if(printLinkTable()) echo '<table><tr>
      <td>&nbsp;&nbsp;&nbsp;<a href="control.php">Control</a>&nbsp;&nbsp;&nbsp;</td>

      <td>&nbsp;&nbsp;&nbsp;<a href="print.php">Print</a>&nbsp;&nbsp;&nbsp;</td>
 
    <td>&nbsp;&nbsp;&nbsp;<a href="http://reprap.org/wiki/RepRapPro_RepRap_Firmware" target="_blank">Help</a>&nbsp;&nbsp;&nbsp;</td>
 

      <td>&nbsp;&nbsp;&nbsp;<a href="settings.php">Settings</a>&nbsp;&nbsp;&nbsp;</td>
 
    <td>&nbsp;&nbsp;&nbsp;<a href="logout.php">Logout</a>&nbsp;&nbsp;&nbsp;</td>
      
    </tr></table>
  <br><br>'; ?>

   <table border="1"><div align="center">
   
   
   <tr>
   <th colspan="9">Move X Y Z</th>
   </tr>
   
   <tr>
   <td rowspan="2"><button type="button" onclick="return homea()">Home<br>All</button></td>
   <td colspan="4">- mm</td>
   <td colspan="4">+ mm</td>
   </tr>
   
   <tr>
   <td>-100</td>
   <td>-10</td>
   <td>-1</td>
   <td>-0.1</td>
   <td>0.1</td>
   <td>1</td>
   <td>10</td>
   <td>100</td>
   </tr>
   
   <tr>
   <td><button type="button" onclick="return homex()">Home X</button></td>
   <td><button type="button" onclick="return xm100mm()">&lt;- X</button></td>
   <td><button type="button" onclick="return xm10mm()">&lt;- X</button></td>
   <td><button type="button" onclick="return xm1mm()">&lt;- X</button></td>
   <td><button type="button" onclick="return xm01mm()">&lt;- X</button></td>
   <td><button type="button" onclick="return xp01mm()">X -&gt;</button></td>
   <td><button type="button" onclick="return xp1mm()">X -&gt;</button></td>
   <td><button type="button" onclick="return xp10mm()">X -&gt;</button></td>
   <td><button type="button" onclick="return xp100mm()">X -&gt;</button></td>
   </tr>
   
   <tr>
   <td><button type="button" onclick="return homey()">Home Y</button></td>
   <td><button type="button" onclick="return ym100mm()">&lt;- Y</button></td>
   <td><button type="button" onclick="return ym10mm()">&lt;- Y</button></td>
   <td><button type="button" onclick="return ym1mm()">&lt;- Y</button></td>
   <td><button type="button" onclick="return ym01mm()">&lt;- Y</button></td>
   <td><button type="button" onclick="return yp01mm()">Y -&gt;</button></td>
   <td><button type="button" onclick="return yp1mm()">Y -&gt;</button></td>
   <td><button type="button" onclick="return yp10mm()">Y -&gt;</button></td>
   <td><button type="button" onclick="return yp100mm()">Y -&gt;</button></td>
   </tr>
   
   <tr>
   <td><button type="button" onclick="return homez()">Home Z</button></td>
   <td><button type="button" onclick="return zm100mm()">&lt;- Z</button></td>
   <td><button type="button" onclick="return zm10mm()">&lt;- Z</button></td>
   <td><button type="button" onclick="return zm1mm()">&lt;- Z</button></td>
   <td><button type="button" onclick="return zm01mm()">&lt;- Z</button></td>
   <td><button type="button" onclick="return zp01mm()">Z -&gt;</button></td>
   <td><button type="button" onclick="return zp1mm()">Z -&gt;</button></td>
   <td><button type="button" onclick="return zp10mm()">Z -&gt;</button></td>
   <td><button type="button" onclick="return zp100mm()">Z -&gt;</button></td>
   </tr>
    
   </div></table>
   
   <br><br><form name="input" action="gather.asp" method="get">Send a G Code: <input type="text" name="gcode"><input type="submit" value="Send"></form>
   
   <script language="javascript" type="text/javascript">
   
   
   function homea(){ window.location.href = "control.php?gcode=G28";}
   function homex(){ window.location.href = "control.php?gcode=G28%20X0";}
   function homey(){ window.location.href = "control.php?gcode=G28%20Y0";}
   function homez(){ window.location.href = "control.php?gcode=G28%20Z0";}
   
   function xp01mm(){ window.location.href = "control.php?gcode=G91%0AG1%20X0.1%0AG90";}
   function xp1mm(){ window.location.href = "control.php?gcode=G91%0AG1%20X1%0AG90";}
   function xp10mm(){ window.location.href = "control.php?gcode=G91%0AG1%20X10%0AG90";}
   function xp100mm(){ window.location.href = "control.php?gcode=G91%0AG1%20X100%0AG90";}
   
   function xm01mm(){ window.location.href = "control.php?gcode=G91%0AG1%20X-0.1%0AG90";}
   function xm1mm(){ window.location.href = "control.php?gcode=G91%0AG1%20X-1%0AG90";}
   function xm10mm(){ window.location.href = "control.php?gcode=G91%0AG1%20X10%0AG90";}
   function xm100mm(){ window.location.href = "control.php?gcode=G91%0AG1%20X100%0AG90";}
   
   function yp01mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Y0.1%0AG90";}
   function yp1mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Y1%0AG90";}
   function yp10mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Y10%0AG90";}
   function yp100mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Y100%0AG90";}
   
   function ym01mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Y-0.1%0AG90";}
   function ym1mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Y-1%0AG90";}
   function ym10mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Y10%0AG90";}
   function ym100mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Y100%0AG90";}
   
   function zp01mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Z0.1%0AG90";}
   function zp1mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Z1%0AG90";}
   function zp10mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Z10%0AG90";}
   function zp100mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Z100%0AG90";}
   
   function zm01mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Z-0.1%0AG90";}
   function zm1mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Z-1%0AG90";}
   function zm10mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Z10%0AG90";}
   function zm100mm(){ window.location.href = "control.php?gcode=G91%0AG1%20Z100%0AG90";}
   
   </script> 
<br><br></html>
