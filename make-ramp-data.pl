# ramp up values from 0 to 255
# in increasing bands over 2000 lines
$runningCount = 1;
$maxCount = 255;
$maxLines = 4000;
$div = $maxLines/$maxCount;
$runningDiv = $div;
# print "$div\n";
for($i=1; $i<=$maxLines; $i++) {
	if($i>$runningDiv){
		$runningDiv += $div;	
		$runningCount++;
	}
	$val = $runningCount;
	print "139U, 139U, 139U,\n";
}
