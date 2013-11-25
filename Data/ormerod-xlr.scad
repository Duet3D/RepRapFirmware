rotate([-90,0,0])
difference()
{
union()
{
cube([30, 5, 35], center=true);


translate([-17, -7.5, 0])
cube([5, 20, 35], center=true);
}

translate([-16.7/2, 0, -26.2/2])
	rotate([90,0,0])
		cylinder(r=1.7, h=40, center=true, $fn=30);

translate([16.7/2, 0, 26.2/2])
	rotate([90,0,0])
		cylinder(r=1.7, h=40, center=true, $fn=30);

rotate([90,0,0])
	cylinder(r=24/2, h=40, center=true, $fn=30);


translate([-15.5, -8, 10])
	rotate([0,90,0])
{
	cylinder(r=2.2, h=40, center=true, $fn=30);
	cylinder(r1=2.2, r2 = 4, h=2.5, center=true, $fn=30);
}

translate([-15.5, -12, -10])
	rotate([0,90,0])
{
	cylinder(r=2.2, h=40, center=true, $fn=30);
	cylinder(r1=2.2, r2 = 4, h=2.5, center=true, $fn=30);
}
}


