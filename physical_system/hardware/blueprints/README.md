# Blueprints :hammer_and_wrench:
This folder contains information about the hardware for all the different versions of Maggy.

## PCB Designs
The PCB design and production files can be found in each folder. Starting with version 4, it is also possible to view the designs and place orders through EasyEDA. Please refer to the specific folders for more information.

## Other Components
Each version of Maggy uses additional components that must be purchased separately from the PCB files.

### Teensy 4.1
We use the Teensy 4.1 development board, specifically this one:
- [Teensy 4.1 on Elfa Distrelec](https://www.elfadistrelec.no/no/teensy-utviklingskort-med-headers-sparkfun-electronics-dev-16996/p/30216161?ext_cid=shgooaqnono-Shopping-PerformanceMax-CSS&cq_src=google_ads&cq_cmp=20351270785&cq_con=&cq_term=&cq_med=pla&cq_plac=&cq_net=x&cq_pos=&cq_plt=gp&gclid=CjwKCAjw4P6oBhBsEiwAKYVkq0jYkFst_neMMFQlXfZBW9-KxBMFTjGZog-tAYaTsAmTVziUdTWCHRoCGOcQAvD_BwE&gclsrc=aw.ds)
- [Teensy 4.1 on RS Online](https://no.rs-online.com/web/p/microcontroller-development-tools/2836911?cm_mmc=NO-PLA-DS3A-_-google-_-CSS_NO_EN_Pmax_Test-_--_-2836911&matchtype=&&gad_source=1&gclid=CjwKCAiA74G9BhAEEiwA8kNfpfuTP6LMEUCaeJ0THdqRm6JLBaF8OzCmbcy2USRwfd5TNwAT9TSBZxoCiPkQAvD_BwE&gclsrc=aw.ds)

If you want to avoid a lot of soldering, ensure that you order with the headers pre-soldered.

### Electromagnets
The electromagnets are standard 19×12 mm copper coils with around 480 windings. Searching for “19x12 maglev electromagnet” should result in many options. For example:
- [Electromagnet on AliExpress](https://www.aliexpress.com/item/4000076659177.html?spm=a2g0o.productlist.main.1.455ew7Phw7PhcV&algo_pvid=489107c2-cb2e-4235-b999-33f5bcec5149&algo_exp_id=489107c2-cb2e-4235-b999-33f5bcec5149-0&pdp_npi=4%40dis%21EUR%216.78%216.24%21%21%217.18%216.61%21%402103249617062671537431660e604c%2110000015610528766%21sea%21NO%210%21AB&curPageLogUid=W7mtjk8XbjMA&utparam-url=scene%3Asearch%7Cquery_from%3A)
- [Electromagnet on Amazon (US)](https://www.amazon.com/Magnetic-Levitation-Electromagnetic-Induction-Experiment/dp/B0B92X4MB2?th=1)
- [Electromagnet on Amazon (DE)](https://www.amazon.de/-/en/Magnetic-Floating-Electromagnetic-Induction-Experiment/dp/B0B8Z66W5S)

### Permanent Magnets for the Base
For the base, we use neodymium magnets with counterbore holes that are fastened with screws to the PCB. The choice of magnet is flexible, but ensure that the dimensions are similar to the ones listed below. The stronger the magnet, the better (i.e., N40 > N35). Also, all magnets on a single unit should have the same polarity, so it’s advisable to purchase them from the same supplier.

Examples of magnets we have used:
- [20×4 mm Magnet with Mounting Hole (Supermagneter)](https://supermagneter.no//magnet-med-monteringshull-20x4mm-id794)
- [20.0×4.5×4.0 mm Neodymium Ring Magnet (Magnet-Shop)](https://www.magnet-shop.com/neodymium/ringmagnets/ringmagnet-20.0x4.5x4.0-mm-n42-nickel-counterbore-n-pole)

### Disk Magnets
The choice of disk magnets is even more flexible, but the examples below are good options. Thicker and heavier magnets have a higher moment of inertia, which makes them easier to stabilize.
- [50×5 mm Disk Magnet (Supermagneter)](https://supermagneter.no//diskmagnet-50x5mm-id752)
- [50.0×4.0 mm Neodymium Disk Magnet (Magnet-Shop)](https://www.magnet-shop.com/neodymium/discmagnets/discmagnet-50.0-x-4.0-mm-n40-gold-holds-21-kg)

### Misc
Screws, spacers and nuts are all standard 3 mm machine screws that should be available everywhere. If possible, use non-magnetic screws. In particular, we have used these spacers for space between PCB and covers/box:
- [M3 spacer, 15mm, male-female](https://www.digikey.no/no/products/detail/w%C3%BCrth-elektronik/971150354/9488642)
- [M3 spacer, 15mm, female-female](https://www.digikey.no/no/products/detail/w%C3%BCrth-elektronik/970150354/9488567)

Also consider that you might need [USB cables](https://www.digikey.no/no/products/detail/cvilux-usa/DH-20M50057/13177527) for connecting to a computer, some antistatic insulation and cardboard boxes for storage, and laser cut protective plates/boxes for protection (we have used 3 mm acrylic plates, cut with blueprints found in the folders).
