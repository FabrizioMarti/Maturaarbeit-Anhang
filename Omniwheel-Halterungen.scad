//Halterungen Kugellager-Omniwheel
/*
union()
{
translate ([0,0,4.9])
cylinder(r=4,h=32,$fn=160);
cylinder(r=5,h=5,$fn=160);
}


translate ([20,0,0])
{
union()
{
translate ([0,0,4.9])
cylinder(r=4,h=32,$fn=160);
cylinder(r=5,h=5,$fn=160);
}
}
*/

/*
translate([0,40,0])
cylinder(r=5,h=5,$fn=160);
translate ([0,40,5])
cylinder (r=2,h=5,$fn=160);
*/

difference()
{
translate ([0,20,17.5])
cylinder (r=4,h=20,$fn=160);

translate ([0,20,32.6])
cylinder (r=2.5,h=5,$fn=160);
}


difference ()
{
translate ([0,20,0])
cylinder (r=4,h=17.5,$fn=160);

translate ([0,-20,-0.1])
{
difference()
{
translate ([0,40,0])
cylinder (r=2.6,h=10,$fn=160);
    
translate ([2.6,40,4])
cube ([1.6,5,8],center=true);
}
}
}




translate([20,00,0])
{

//translate([0,40,0])
//cylinder(r=5,h=5,$fn=160);

//translate ([0,40,5])
//cylinder (r=2,h=5,$fn=160);

difference()
{
translate ([0,20,17.5])
cylinder (r=4,h=20,$fn=160);

translate ([0,20,32.6])
cylinder (r=2.5,h=5,$fn=160);
}


difference ()
{
translate ([0,20,0])
cylinder (r=4,h=17.5,$fn=160);

translate ([0,-20,-0.1])
{
difference()
{
translate ([0,40,0])
cylinder (r=2.6,h=10,$fn=160);
    
translate ([2.6,40,4])
cube ([1.6,5,8],center=true);
}
}
}

}
