// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Sensor/ImprovedGnssSensor.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/MapGen/LargeMapManager.h"

#include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"
#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Vector3D.h"
#include <compiler/enable-ue4-macros.h>
#include <cmath>

#define persistence 0.9

AImprovedGnssSensor::AImprovedGnssSensor(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
}

FActorDefinition AImprovedGnssSensor::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeImprovedGnssDefinition();
}

void AImprovedGnssSensor::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetImprovedGnss(ActorDescription, this);
}

void AImprovedGnssSensor::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AImprovedGnssSensor::PostPhysTick);

  //Calculated Position
  std::map<int, AImprovedGnssSensor::satellite_t> DetectedSatellites = GetVisibleSatellites();
  std::map<int, double> Pseudoranges = GetPseudoranges(DetectedSatellites);
  FVector ActorLocation = GetVectorLocation(DetectedSatellites, Pseudoranges);

  carla::geom::Location Location = ActorLocation;
  carla::geom::GeoLocation CurrentLocation = CurrentGeoReference.Transform(Location);

  //True Position
  FVector TrueActorLocation = GetActorLocation();
  ALargeMapManager * LargeMap = UCarlaStatics::GetLargeMapManager(GetWorld());
  if (LargeMap)
  {
    TrueActorLocation = LargeMap->LocalToGlobalLocation(TrueActorLocation);
  }

  carla::geom::Location TrueLocation = TrueActorLocation;
  carla::geom::GeoLocation TrueCurrentLocation = CurrentGeoReference.Transform(TrueLocation);

  LatitudeBias = persistence*LatitudeBias + (1-persistence)*(CurrentLocation.latitude -TrueCurrentLocation.latitude);
  LongitudeBias = persistence*LongitudeBias + (1-persistence)*(CurrentLocation.longitude -TrueCurrentLocation.longitude);
  AltitudeBias = persistence*AltitudeBias + (1-persistence)*(CurrentLocation.altitude -TrueCurrentLocation.altitude);
  LatitudeDeviation = persistence*LatitudeDeviation + (1-persistence)*((CurrentLocation.latitude -TrueCurrentLocation.latitude)-LatitudeBias)*((CurrentLocation.latitude -TrueCurrentLocation.latitude)-LatitudeBias);
  LongitudeDeviation = persistence*LongitudeDeviation + (1-persistence)*((CurrentLocation.longitude -TrueCurrentLocation.longitude)-LongitudeBias)*((CurrentLocation.longitude -TrueCurrentLocation.longitude)-LongitudeBias);
  AltitudeDeviation = persistence*AltitudeDeviation + (1-persistence)*((CurrentLocation.altitude -TrueCurrentLocation.altitude)-AltitudeBias)*((CurrentLocation.altitude -TrueCurrentLocation.altitude)-AltitudeBias);

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("AImprovedGnssSensor Stream Send");
    auto Stream = GetDataStream(*this);
    Stream.Send(*this, CurrentLocation);
  }
}

void AImprovedGnssSensor::SetLatitudeDeviation(float Value)
{
  return;
}

void AImprovedGnssSensor::SetLongitudeDeviation(float Value)
{
  return;
}

void AImprovedGnssSensor::SetAltitudeDeviation(float Value)
{
  return;
}

void AImprovedGnssSensor::SetLatitudeBias(float Value)
{
  return;
}

void AImprovedGnssSensor::SetLongitudeBias(float Value)
{
  return;
}

void AImprovedGnssSensor::SetAltitudeBias(float Value)
{
  return;
}

std::map<int, double> AImprovedGnssSensor::GetPseudoranges(std::map<int, AImprovedGnssSensor::satellite_t> DetectedSatellites)
{
  std::map<int, double> Pseudoranges;

  FVector ActorLocation = GetActorLocation();
  ALargeMapManager * LargeMap = UCarlaStatics::GetLargeMapManager(GetWorld());
  if (LargeMap)
  {
    ActorLocation = LargeMap->LocalToGlobalLocation(ActorLocation);
  }

  FVector SatelliteLocation;
  for (std::pair<int, AImprovedGnssSensor::satellite_t> currentSatellite : DetectedSatellites)
  {
    SatelliteLocation = currentSatellite.second.location;
    if (LargeMap)
    {
      SatelliteLocation = LargeMap->LocalToGlobalLocation(SatelliteLocation);
    }
    Pseudoranges.insert(std::pair<int, double>(currentSatellite.first,FVector::Dist(SatelliteLocation,ActorLocation)));
  }
  return Pseudoranges;
}

std::map<int, AImprovedGnssSensor::satellite_t> AImprovedGnssSensor::GetVisibleSatellites()
{
  std::map<int, AImprovedGnssSensor::satellite_t> DetectedSatellites;

  std::vector<std::map<std::string, double>> sat_ephem = ReadSatellitesFromCsv();

  double t_in = UGameplayStatics::GetRealTimeSeconds(GetWorld());

  for(std::map<std::string, double> sat_data : sat_ephem)
  {
    AImprovedGnssSensor::satellite_t sat = calculateSatPos(sat_data, t_in);
    DetectedSatellites[sat.PRN] = sat;
  }
  return DetectedSatellites;
}

std::vector<std::map<std::string, double>> AImprovedGnssSensor::ReadSatellitesFromCsv()
{
  std::vector<std::map<std::string, double>> sat_ephem_out;
  std::vector<std::string> colnames;
  std::string line, colname;
  double val;

  std::string filename = "satellite_ephem.csv";
  std::ifstream myFile(filename);

  if(myFile.is_open())
  {

    if(myFile.good())
    {
      std::getline(myFile, line);
      std::stringstream ss(line);
      while(std::getline(ss, colname, ','))
      {
        colnames.push_back(colname);
      }
    }

    while(std::getline(myFile, line))
    {
      std::stringstream ss(line);
      
      int colIdx = 0;
      std::map<std::string, double> sat_ephem;
      while(ss >> val)
      {
        sat_ephem[colnames[colIdx]] = val;
        if(ss.peek() == ',') ss.ignore();
        colIdx++;
      }
      sat_ephem_out.push_back(sat_ephem);
    }

    myFile.close();

  }

  return sat_ephem_out;
}

AImprovedGnssSensor::satellite_t AImprovedGnssSensor::calculateSatPos(std::map<std::string, double> s, double t_in)
{
  AImprovedGnssSensor::satellite_t sat;

  double a = pow(s["sqrtA"],2);
  double n = sqrt(mu/pow(a,3)) + s["deltaN"];
  double e = s["Eccentricity"];

  double tk = t_in - s["Toe"];
  double Mk = s["M0"] + n*tk;
  double Ek = NewtonRaphsonE(Mk,1e-9,e);
  double s_vk = (sqrt(1-pow(e,2))*sin(Ek))/(1-e*cos(Ek));
  double c_vk = (cos(Ek) - e)/(1-e*cos(Ek));
  double vk = atan2(s_vk,c_vk);
  double phik = vk + s["omega"];

  double uk = phik;
  for (int i = 0; i < 50; i++) 
  {
    double dphik = s["Cus"]*sin(2*uk) + s["Cuc"]*cos(2*uk);
    uk = phik + dphik;
  }

  double drk = s["Crs"]*sin(2*phik) + s["Crc"]*cos(2*phik);
  double dik = s["Cis"]*sin(2*phik) + s["Cic"]*cos(2*phik);
  double Omk = s["Omega0"] - Om_dot_e*t_in + s["OmegaDot"]*tk;
  double rk = a*(1 - e*cos(Ek)) + drk;
  double ik = s["Io"] + s["IDOT"]*tk + dik;
  double xp = rk*cos(uk);
  double yp = rk*sin(uk);
  double x_ecef = xp*cos(Omk) - yp*cos(ik)*sin(Omk);
  double y_ecef = xp*sin(Omk) + yp*cos(ik)*cos(Omk);
  double z_ecef = yp*sin(ik);

  // double tdiff = (Nw - s["GPSWeekToc"])*604800 + (t_in - s["Toc"]);
  sat.bias = 0; //c*(s["af0"]+s["af1"]*tdiff+s["af2"]*pow(tdiff,2) + F*e*s["sqrtA"]*sin(Ek));
  sat.PRN = s["PRN"];
  sat.location = AImprovedGnssSensor::ConvertToGlobalVector(x_ecef,y_ecef,z_ecef);


  return sat;
}

double AImprovedGnssSensor::NewtonRaphsonE(double M, double epsilon, double e)
{
  double E;

  if(e <.5){
    E = pi;
  }
  else
  {
    E = M;
  }
  
  for (int i = 0; i < 50; i++)
  {
    double d = (E-e*sin(E)-M)/(1-e*cos(E));
    
    if (abs(d) >= epsilon)
    {
      E = E - d;
    }
  }
  
  return E;
}

FVector AImprovedGnssSensor::ConvertToGlobalVector(double x_ecef, double y_ecef, double z_ecef)
{
  carla::geom::GeoLocation RefLocation = CurrentGeoReference;
  double lat = RefLocation.latitude*pi/180.0;
  double lon = RefLocation.longitude*pi/180.0;
  double h = RefLocation.altitude;
  double Rp = Re*sqrt(1 - pow(eE,2));
  double N = Re/sqrt(1 - pow(eE,2)*pow(sin(lat),2));
  double Ref_x_ECEF = (N+h)*cos(lat)*cos(lon);
  double Ref_y_ECEF = (N+h)*cos(lat)*sin(lon);
  double Ref_z_ECEF = (N*(1 - pow(eE,2))+h)*sin(lat);

  double dx_ECEF = x_ecef - Ref_x_ECEF;
  double dy_ECEF = y_ecef - Ref_y_ECEF;
  double dz_ECEF = z_ecef - Ref_z_ECEF;

  double E = -sin(lon)*dx_ECEF + cos(lon)*dy_ECEF;
  double S = -(-sin(lat)*cos(lon)*dx_ECEF + -sin(lat)*sin(lon)*dy_ECEF + cos(lat)*dz_ECEF);
  double U = cos(lat)*cos(lon)*dx_ECEF + cos(lat)*sin(lon)*dy_ECEF + sin(lat)*dz_ECEF;

  return FVector(E,S,U);
}

FVector AImprovedGnssSensor::GetVectorLocation(std::map<int, AImprovedGnssSensor::satellite_t> DetectedSatellites, std::map<int, double> Pseudoranges)
{
  double p[DetectedSatellites.size()];
  FVector S[DetectedSatellites.size()];
  double B[DetectedSatellites.size()];
  FVector SatelliteLocation;
  FVector ActorLocation = GetActorLocation();

  int i = 0;
  for (std::pair<int, AImprovedGnssSensor::satellite_t> currentSatellite : DetectedSatellites)
  {
    SatelliteLocation = currentSatellite.second.location;

    p[i] = Pseudoranges.at(currentSatellite.first);
    S[i] = SatelliteLocation;
    B[i] = 0;
    i++;
  }

  NewtonRhapson(p, S, B, 1e-3);

  return FVector(CurrentVecReference.X,CurrentVecReference.Y,CurrentVecReference.Z);
}

bool AImprovedGnssSensor::NewtonRhapson(double p[], FVector S[], double B[], double epsilon) 
{
  FVector ydiff = FVector4(0);
  FVector4 yb = CurrentVecReference;
  for(int i = 0; i <= 100; i++)
  {
    ydiff = positionUpdate(yb,p,S,B);
    yb = yb + ydiff;
    if(ydiff.Size() < epsilon)
    {
      CurrentVecReference = yb;
      return true;
    }
  }
  return false;
}
FVector AImprovedGnssSensor::positionUpdate(FVector4 yb, double p[], FVector S[], double B[]) 
{
  double dp[sizeof(*S)];
  FVector l;
  FVector4 G[sizeof(*S)];
  FVector4 Gdp = FVector4(0);
  double Gsq[4][4];
  double invGsq[4][4];
  FVector diff;

  FVector y = FVector(yb.X,yb.Y,yb.W);
  for (int i = 0; i < sizeof(*S); i++)
  {
    dp[i] = p[i] - FVector::Dist(y,S[i]) + yb.W - B[i];
    diff = S[i] - y;
    l = diff.GetSafeNormal(0.000001);
    G[i] = FVector4(-l.X,-l.Y,-l.Z,1);
    Gdp = G[i]*dp[i]+Gdp;
    for (int j = 0; j < 4; j++)
    {
      for (int k = 0; k < 4; k++)
      {
        Gsq[j][k] = G[i][j]*G[i][k] + Gsq[j][k];
      }
    }
  }
  inverse(Gsq, invGsq);
  FVector4 dy = FVector4(0);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      dy[i] = invGsq[i][j]*Gdp[j] + dy[i];
    }
  }
  return dy;
}

// Derived from code at https://www.geeksforgeeks.org/adjoint-inverse-matrix/
// Beginning of section
void AImprovedGnssSensor::getCofactor(double A[DIM][DIM], double temp[DIM][DIM], int p, int q, int n)
{
    int i = 0, j = 0;
    for (int row = 0; row < n; row++)
    {
        for (int col = 0; col < n; col++)
        {
            if (row != p && col != q)
            {
                temp[i][j++] = A[row][col];
                if (j == n - 1)
                {
                    j = 0;
                    i++;
                }
            }
        }
    }
}
double AImprovedGnssSensor::determinant(double A[DIM][DIM], int n)
{
    double D = 0; 
    if (n == 1)
    {
        return A[0][0];
    }
    double temp[DIM][DIM]; 
    int sign = 1; 

    for (int f = 0; f < n; f++)
    {
        getCofactor(A, temp, 0, f, n);
        D += sign * A[0][f] * determinant(temp, n - 1);
        sign = -sign;
    }
    return D;
}
void AImprovedGnssSensor::adjoint(double A[DIM][DIM],double adj[DIM][DIM])
{
    if (DIM == 1)
    {
        adj[0][0] = 1;
        return;
    }
    int sign = 1;
    double temp[DIM][DIM];
    for (int i=0; i<DIM; i++)
    {
        for (int j=0; j<DIM; j++)
        {
            getCofactor(A, temp, i, j, DIM);
            sign = ((i+j)%2==0)? 1: -1;
            adj[j][i] = (sign)*(determinant(temp, DIM-1));
        }
    }
}
bool AImprovedGnssSensor::inverse(double A[DIM][DIM], double inverse[DIM][DIM])
{
    double det = determinant(A, DIM);
    if (det == 0)
    {
        return false;
    }
    double adj[DIM][DIM];
    adjoint(A, adj);
    for (int i=0; i<DIM; i++)
        for (int j=0; j<DIM; j++)
            inverse[i][j] = adj[i][j]/double(det);
 
    return true;
}
// End of section

void AImprovedGnssSensor::BeginPlay()
{
  Super::BeginPlay();

  const UCarlaEpisode* episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
  CurrentGeoReference = episode->GetGeoReference();
}
