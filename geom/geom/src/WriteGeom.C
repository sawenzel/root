void WriteGeom(const char* volumefile, const char* volumename, const char* finalfile) {
   auto fin = TFile::Open(volumefile);
   if (!fin) {
    return;
   }
   auto obj = fin->Get(volumename);
   if (!obj) {
    return;
   }
   auto shape = dynamic_cast<TGeoShape*>(obj);
   if (!shape) {
    return;
   }
   
   auto *airMat = new TGeoMaterial("AIR", 14.7, 7.3, 0.001225);
   auto *airMed = new TGeoMedium("AIR", 1, airMat);

   TGeoVolume worldv("foo", shape, airMed);
   gGeoManager->SetTopVolume(&worldv);
   gGeoManager->Export(finalfile);
}