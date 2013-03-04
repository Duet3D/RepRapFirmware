<!DOCTYPE HTML>
<head>
<style type="text/css">td { text-align: left; } </style>
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

<form onSubmit="return checkFileName(this)" action="print.php"
enctype="multipart/form-data" method="post">
<p>
Upload a G Code file :<br>
<input type="file" name="datafile" size="40">
</p>
<div>
<input type="submit" value="Upload">
</div>
</form>

<br><br>Click a file to print it:
<br><br>

<script language="javascript" type="text/javascript">
function fileList()
{
	var files = [<?php print(getGCodeList()); ?>];
	return files;
}


function printGCodeTable()
{
  var list = fileList();

  var count = list.length;
  
  if(count <= 0)
  	return "<br>No GCode files present.<br>";

  
  var cols = Math.floor(Math.sqrt(count)) + 1;
  var rows = Math.floor(count/cols) + 1;
  
  var result = "<table>";
  
  var k = 0;

  for(var i = 0; i < cols; i++)
  {
    result += "<tr>";
    for(var j = 0; j < rows; j++)
    {
      var fileName = list[i*rows + j];
      result += "<td>&nbsp;<button type=\"button\" onclick=\"return printFile('";
      result += "gcodes/" + fileName; // Need PHP in here
      result += "')\">";
      result += fileName;
      result += "</button>&nbsp;</td>";
      k++;
      if(k >= count)
        break;
    }
    result += "</tr>";
    if(k >= count)
        break;
  }
  result += "</table>";
  return result;
}


document.write(printGCodeTable());
</script>
<br><br>


<br><br>
<button type="button" onclick="return pausePrint()">Pause the print</button>
<br><br>
<hr>
<br>
<button type="button" onclick="return deleteFile()">Delete a file</button>



<script language="javascript" type="text/javascript">
   
function printFile(filetoprint){ window.location.href = "print.php?gcode=M23%20" + filetoprint + "%0AM24";}

function pausePrint(){ window.location.href = "print.php?gcode=M25";}

function deleteFile(){ window.location.href = "delete.php";}

function checkFileName(uploadForm)
{

  var list = fileList();
  for(var i = 0; i < list.length; i++)
  {
	if(list[i].toUpperCase() == uploadForm.datafile.value.toUpperCase())
	{
		return confirm("This will overwrite the file " + 
			list[i] + " on  <?php print(getMyName()); ?>." +
				"Continue?");
	}
  }

return true;

}


</script> 

</html>

