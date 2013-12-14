difference()
{
translate([0,11-8.8,0])
difference()
{
cube([26,22,7],center=true);
translate([13.5,4,0])
cube([20,20,8],center=true);
translate([-13.5,4,0])
cube([20,20,8],center=true);

}
translate([0,10,0])
cylinder(r=1.7, h=40, center=true, $fn=30);
cylinder(r=1.7, h=40, center=true, $fn=30);
translate([7.5,0,0])
rotate([90,0,0])
cylinder(r=1.3, h=40, center=true, $fn=30);
translate([-7.5,0,0])
rotate([90,0,0])
cylinder(r=1.3, h=40, center=true, $fn=30);
}
