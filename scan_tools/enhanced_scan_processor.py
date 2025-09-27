#!/usr/bin/env python3
"""
Enhanced Scan Processor for AUAS Inspection Engine

This processor:
1. Scans inspection folders for new PLY scan data (with date filtering)
2. Cleans each PLY file using optimized parameters
3. Compares cleaned scans with reference PLY files 
4. Generates analysis results and reports
"""
from __future__ import annotations
from pathlib import Path
import argparse
import re
import sys
import os
from datetime import datetime
from typing import Iterable, List, Tuple, Dict, Optional

import numpy as np
import open3d as o3d

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
TRACK_FILE = "last_processed_date.txt"

class EnhancedScanProcessor:
    def __init__(self, ftp_root: Path):
        self.ftp_root = Path(ftp_root)
        self.piece_reference_root = self.ftp_root / "piece_reference"
        
        # Optimized cleaning parameters (from your test results)
        self.clean_params = {
            "voxel_mm": 0.4,
            "plane_dist_mm": 1.8,
            "outlier_std_ratio": 2.8,
            "dbscan_eps_mm": 2.8,
            "dbscan_min_points": 6,
            "remove_plane": True,
            "keep_above": True,
            "largest_cluster": True
        }
        
    def list_date_folders(self) -> List[Path]:
        """Get all inspection date folders"""
        insp = self.ftp_root / "inspections"
        if not insp.exists():
            return []
        return sorted([p for p in insp.iterdir() if p.is_dir() and DATE_RE.match(p.name)])

    def read_last_date(self) -> str | None:
        """Read the last processed date from tracking file"""
        f = self.ftp_root / TRACK_FILE
        if not f.exists():
            return None
        try:
            s = f.read_text(encoding="utf-8").strip()
            return s or None
        except Exception:
            return None

    def write_last_date(self, date_str: str) -> None:
        """Write the last processed date to tracking file"""
        (self.ftp_root / TRACK_FILE).write_text(date_str + "\n", encoding="utf-8")

    def should_process(self, date_name: str, last_date: str | None) -> bool:
        """Check if a date folder should be processed"""
        if last_date is None:
            return True
        return date_name > last_date

    def clean_point_cloud(self, base_pcd: o3d.geometry.PointCloud) -> Tuple[o3d.geometry.PointCloud, Dict[str, int]]:
        """Apply the EXACT same cleaning pipeline as scan_viewer.clean_preview() - COPY/PASTE"""
        stats = {"original": len(base_pcd.points)}
        print(f"    🔧 Starting cleaning with {len(base_pcd.points)} points (EXACT clean_preview COPY)")

        try:
            # EXACT COPY OF THE CLEANING PART from clean_preview() - NO CHANGES AT ALL
            
            # Parameters - USE EXACT SAME VALUES/UNITS AS VIEWER
            # NOTE: scan_viewer treats all numeric fields as millimeters and
            # passes them directly to Open3D without converting to meters.
            # To match results exactly, do NOT divide by 1000 here.
            voxel = float(self.clean_params["voxel_mm"])            # mm
            plane_dist = float(self.clean_params["plane_dist_mm"])   # mm
            out_std = self.clean_params["outlier_std_ratio"]
            db_eps = float(self.clean_params["dbscan_eps_mm"])      # mm
            db_min = self.clean_params["dbscan_min_points"]
            depth = 9  # same as viewer

            # EXACT COPY FROM clean_preview() - START
            pcd = base_pcd.voxel_down_sample(max(1e-6, float(voxel))) if voxel > 1e-6 else base_pcd
            if len(pcd.points) == 0:
                print("    ❌ No points after voxel downsample.")
                return pcd, stats
            
            stats["after_voxel"] = len(pcd.points)
            print(f"    📐 After voxel: {len(pcd.points)} points")
            
            # Remove sparse outliers first
            try:
                pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=float(out_std))
                stats["after_outlier_removal"] = len(pcd.points) 
                print(f"    🧹 After outlier removal: {len(pcd.points)} points")
            except Exception:
                pass

            # Estimate normals for later steps
            try:
                pcd.estimate_normals()
                print("    🧭 Normals estimated")
            except Exception:
                pass

            # Plane segmentation and above-plane filtering - EXACT LOGIC
            if self.clean_params["remove_plane"] or self.clean_params["keep_above"]:
                try:
                    model, inliers = pcd.segment_plane(distance_threshold=float(plane_dist), ransac_n=3, num_iterations=500)
                    a, b, c, d = model
                    n = np.array([a, b, c], dtype=np.float64)
                    nn = np.linalg.norm(n)
                    if nn > 0:
                        n = n / nn
                        d = d / nn
                    # Orient normal upwards (Z positive)
                    if n[2] < 0:
                        n = -n
                        d = -d
                    pts_np = np.asarray(pcd.points)
                    print(f"    🌍 Plane found: normal={n}, d={d}, inliers={len(inliers)}")
                    
                    # EXACT COPY: Remove plane points first
                    if self.clean_params["remove_plane"]:
                        mask = np.ones((pts_np.shape[0],), dtype=bool)
                        mask[np.asarray(inliers, dtype=int)] = False
                        pts_np = pts_np[mask]  # UPDATE pts_np here!
                        if pcd.has_colors():
                            cols_np = np.asarray(pcd.colors)[mask]
                        else:
                            cols_np = None
                        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_np))
                        if cols_np is not None:
                            pcd.colors = o3d.utility.Vector3dVector(cols_np)
                        print(f"    🌍 Removed plane points: {len(pts_np)} remaining")
                    
                    # EXACT COPY: Keep only above plane using UPDATED pts_np
                    if self.clean_params["keep_above"] and pts_np.shape[0] > 0:
                        signed = np.dot(pts_np, n) + d
                        keep = signed > 0.0
                        pts_np = pts_np[keep]  # Use the already updated pts_np!
                        if pcd.has_colors():
                            cols_np = np.asarray(pcd.colors)
                            cols_np = cols_np[keep]
                        else:
                            cols_np = None
                        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_np))
                        if cols_np is not None:
                            pcd.colors = o3d.utility.Vector3dVector(cols_np)
                        print(f"    🌍 Kept points above plane: {len(pts_np)} points")
                        
                    stats["after_plane_removal"] = len(pcd.points)
                except Exception as e:
                    print(f"    ⚠️ Plane segmentation failed: {e}")

            # Largest cluster selection - EXACT COPY
            if self.clean_params["largest_cluster"]:
                try:
                    labels = np.array(pcd.cluster_dbscan(eps=float(db_eps), min_points=int(db_min), print_progress=False))
                    if labels.size and labels.max() >= 0:
                        counts = np.bincount(labels[labels >= 0])
                        largest_label = counts.argmax()
                        mask = labels == largest_label
                        pts_np = np.asarray(pcd.points)[mask]
                        if pcd.has_colors():
                            cols_np = np.asarray(pcd.colors)[mask]
                        else:
                            cols_np = None
                        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_np))
                        if cols_np is not None:
                            pcd.colors = o3d.utility.Vector3dVector(cols_np)
                        print(f"    🔗 Largest cluster (label {largest_label}): {len(pts_np)} points")
                    else:
                        print(f"    🔗 No valid clusters found!")
                    stats["after_clustering"] = len(pcd.points)
                except Exception as e:
                    print(f"    ⚠️ Clustering failed: {e}")

            # EXACT COPY FROM clean_preview() - END
            
            stats["final"] = len(pcd.points)
            print(f"    ✅ Cleaning complete: {stats['original']} → {stats['final']} points")
            
            return pcd, stats
            
        except Exception as e:
            print(f"    ❌ Clean point cloud failed: {e}")
            return base_pcd, stats

    def compare_with_reference(self, cleaned_pcd: o3d.geometry.PointCloud, 
                              reference_pcd: o3d.geometry.PointCloud) -> Dict[str, float]:
        """Compare cleaned scan with reference and generate analysis metrics"""
        
        try:
            # Basic statistics
            scan_points = np.asarray(cleaned_pcd.points)
            ref_points = np.asarray(reference_pcd.points)
            
            # Point count comparison
            point_count_ratio = len(scan_points) / len(ref_points) if len(ref_points) > 0 else 0
            
            # Bounding box comparison
            scan_bounds = [scan_points.min(axis=0), scan_points.max(axis=0)]
            ref_bounds = [ref_points.min(axis=0), ref_points.max(axis=0)]
            
            scan_volume = np.prod(scan_bounds[1] - scan_bounds[0])
            ref_volume = np.prod(ref_bounds[1] - ref_bounds[0])
            volume_ratio = scan_volume / ref_volume if ref_volume > 0 else 0
            
            # Centroid distance
            scan_centroid = scan_points.mean(axis=0)
            ref_centroid = ref_points.mean(axis=0)
            centroid_distance = np.linalg.norm(scan_centroid - ref_centroid)
            
            # Hausdorff distance (simplified - sample-based for performance)
            if len(scan_points) > 1000:
                scan_sample = scan_points[::len(scan_points)//1000]
            else:
                scan_sample = scan_points
                
            if len(ref_points) > 1000:
                ref_sample = ref_points[::len(ref_points)//1000]
            else:
                ref_sample = ref_points
            
            # Compute approximate Hausdorff distance
            dist_scan_to_ref = []
            for point in scan_sample:
                distances = np.linalg.norm(ref_sample - point, axis=1)
                dist_scan_to_ref.append(distances.min())
            
            dist_ref_to_scan = []
            for point in ref_sample:
                distances = np.linalg.norm(scan_sample - point, axis=1)
                dist_ref_to_scan.append(distances.min())
            
            hausdorff_dist = max(max(dist_scan_to_ref), max(dist_ref_to_scan)) if dist_scan_to_ref and dist_ref_to_scan else 0
            mean_distance = np.mean(dist_scan_to_ref + dist_ref_to_scan)
            
            # Calculate similarity score (0-100%)
            # Based on multiple factors: point count, volume, distances
            point_score = min(point_count_ratio, 2.0) * 0.3  # 30% weight, cap at 2.0 ratio
            volume_score = min(volume_ratio, 2.0) * 0.2      # 20% weight
            distance_score = max(0, 1 - (mean_distance / 10.0)) * 0.5  # 50% weight, assuming 10mm is very bad
            
            similarity_score = (point_score + volume_score + distance_score) * 100
            similarity_score = max(0, min(100, similarity_score))  # Clamp to 0-100%
            
            return {
                "point_count_ratio": point_count_ratio,
                "volume_ratio": volume_ratio,
                "centroid_distance_mm": centroid_distance * 1000,  # Convert to mm
                "hausdorff_distance_mm": hausdorff_dist * 1000,
                "mean_distance_mm": mean_distance * 1000,
                "similarity_score": similarity_score,
                "scan_point_count": len(scan_points),
                "reference_point_count": len(ref_points)
            }
            
        except Exception as e:
            print(f"Error in comparison: {e}")
            return {"error": str(e)}

    def get_piece_reference(self, inspection_folder: Path) -> Optional[str]:
        """Extract piece reference from inspection report or reference.txt"""
        
        # First try to read from the global reference.txt (latest reference)
        ref_file = self.ftp_root / "reference.txt"
        if ref_file.exists():
            ref = ref_file.read_text().strip()
            if ref:
                return ref
        
        # Fallback: try to read from inspection report
        report_file = inspection_folder / "inspection_report.txt"
        if report_file.exists():
            try:
                content = report_file.read_text()
                for line in content.split('\n'):
                    if line.startswith('Reference:'):
                        ref = line.split(':', 1)[1].strip()
                        if ref != 'Unknown':
                            return ref
            except Exception:
                pass
        
        return None

    def find_reference_ply(self, piece_reference: str) -> Optional[Path]:
        """Find the reference PLY file for a piece"""
        ref_folder = self.piece_reference_root / piece_reference
        if not ref_folder.exists():
            return None
        
        # Look for PLY files in the reference folder
        ply_files = list(ref_folder.glob("*.ply"))
        if ply_files:
            # Return the most recent PLY file
            return max(ply_files, key=lambda p: p.stat().st_mtime)
        
        return None

    def process_inspection_folder(self, inspection_folder: Path) -> List[Dict]:
        """Process all PLY files in an inspection folder"""
        results = []
        
        # Find scan data folder
        scan_data_folder = inspection_folder / "scan_data"
        if not scan_data_folder.exists():
            print(f"No scan_data folder in {inspection_folder}")
            return results
        
        # Get piece reference
        piece_reference = self.get_piece_reference(inspection_folder)
        if not piece_reference:
            print(f"No piece reference found for {inspection_folder}")
            return results
        
        # Find reference PLY
        reference_ply_path = self.find_reference_ply(piece_reference)
        reference_pcd = None
        if reference_ply_path:
            try:
                reference_pcd = o3d.io.read_point_cloud(str(reference_ply_path))
                print(f"Loaded reference PLY: {reference_ply_path}")
            except Exception as e:
                print(f"Failed to load reference PLY {reference_ply_path}: {e}")
        
        # Process all PLY files in scan_data
        ply_files = list(scan_data_folder.glob("*.ply"))
        
        for ply_file in ply_files:
            try:
                print(f"Processing: {ply_file}")
                
                # Load original PLY
                original_pcd = o3d.io.read_point_cloud(str(ply_file))
                if len(original_pcd.points) == 0:
                    print(f"Empty PLY file: {ply_file}")
                    continue
                
                # Clean the point cloud
                cleaned_pcd, clean_stats = self.clean_point_cloud(original_pcd)
                
                # Save cleaned PLY
                cleaned_filename = ply_file.stem + "_cleaned.ply"
                cleaned_path = scan_data_folder / cleaned_filename
                
                print(f"    💾 Saving cleaned PLY to: {cleaned_path}")
                # Save explicitly as binary PLY (compact) to match viewer
                success = o3d.io.write_point_cloud(
                    str(cleaned_path), cleaned_pcd, write_ascii=False, compressed=False, print_progress=False
                )
                
                if success:
                    # Verify the saved file
                    verification_pcd = o3d.io.read_point_cloud(str(cleaned_path))
                    print(f"    ✅ Saved and verified: {len(verification_pcd.points)} points in file")
                    
                    if len(verification_pcd.points) != len(cleaned_pcd.points):
                        print(f"    ⚠️ WARNING: Point count mismatch! Expected {len(cleaned_pcd.points)}, got {len(verification_pcd.points)}")
                else:
                    print(f"    ❌ Failed to save PLY file!")
                    continue
                
                result = {
                    "original_ply": ply_file.name,
                    "cleaned_ply": cleaned_filename,
                    "piece_reference": piece_reference,
                    "clean_stats": clean_stats,
                    "timestamp": datetime.now().isoformat()
                }
                
                # Compare with reference if available
                if reference_pcd is not None:
                    comparison = self.compare_with_reference(cleaned_pcd, reference_pcd)
                    result["comparison"] = comparison
                    result["reference_ply"] = reference_ply_path.name
                else:
                    result["comparison"] = {"error": "No reference PLY found"}
                    result["reference_ply"] = None
                
                results.append(result)
                print(f"✅ Processed {ply_file.name} -> {cleaned_filename}")
                
            except Exception as e:
                print(f"❌ Error processing {ply_file}: {e}")
                results.append({
                    "original_ply": ply_file.name,
                    "error": str(e),
                    "timestamp": datetime.now().isoformat()
                })
        
        return results

    def generate_analysis_report(self, inspection_folder: Path, results: List[Dict]):
        """Generate a comprehensive analysis report"""
        
        report_path = inspection_folder / "scan_analysis_report.txt"
        
        try:
            with open(report_path, 'w', encoding='utf-8') as f:
                f.write("AUAS SCAN ANALYSIS REPORT\n")
                f.write("=" * 50 + "\n\n")
                f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Inspection Folder: {inspection_folder.name}\n\n")
                
                # Summary
                total_scans = len(results)
                successful_scans = len([r for r in results if "error" not in r])
                scans_with_comparison = len([r for r in results if "error" not in r and r.get("comparison", {}).get("similarity_score") is not None])
                
                f.write("PROCESSING SUMMARY\n")
                f.write("-" * 30 + "\n")
                f.write(f"Total PLY files found: {total_scans}\n")
                f.write(f"Successfully processed: {successful_scans}\n")
                f.write(f"With reference comparison: {scans_with_comparison}\n\n")
                
                # Cleaning parameters used
                f.write("CLEANING PARAMETERS\n")
                f.write("-" * 30 + "\n")
                for key, value in self.clean_params.items():
                    f.write(f"{key}: {value}\n")
                f.write("\n")
                
                # Individual scan results
                f.write("INDIVIDUAL SCAN RESULTS\n")
                f.write("-" * 30 + "\n")
                
                for i, result in enumerate(results, 1):
                    f.write(f"\n{i}. {result['original_ply']}\n")
                    
                    if "error" in result:
                        f.write(f"   ❌ ERROR: {result['error']}\n")
                        continue
                    
                    f.write(f"   ✅ Cleaned: {result['cleaned_ply']}\n")
                    f.write(f"   📋 Piece Reference: {result['piece_reference']}\n")
                    
                    # Cleaning stats
                    stats = result.get("clean_stats", {})
                    f.write(f"   📊 Point Reduction: {stats.get('original', 0)} → {list(stats.values())[-1] if stats else 0}\n")
                    
                    # Comparison results
                    comparison = result.get("comparison", {})
                    if "error" not in comparison and comparison.get("similarity_score") is not None:
                        f.write(f"   🎯 Similarity Score: {comparison['similarity_score']:.1f}%\n")
                        f.write(f"   📏 Mean Distance: {comparison['mean_distance_mm']:.2f}mm\n")
                        f.write(f"   📐 Centroid Distance: {comparison['centroid_distance_mm']:.2f}mm\n")
                        f.write(f"   📊 Point Count Ratio: {comparison['point_count_ratio']:.2f}\n")
                        
                        # Quality assessment
                        score = comparison['similarity_score']
                        if score >= 85:
                            f.write(f"   ✅ QUALITY: EXCELLENT\n")
                        elif score >= 70:
                            f.write(f"   ⚠️ QUALITY: GOOD\n")
                        elif score >= 50:
                            f.write(f"   ⚠️ QUALITY: ACCEPTABLE\n")
                        else:
                            f.write(f"   ❌ QUALITY: POOR - INVESTIGATION NEEDED\n")
                    else:
                        f.write(f"   ❓ No reference comparison available\n")
                
                # Overall assessment
                if scans_with_comparison > 0:
                    scores = [r["comparison"]["similarity_score"] for r in results 
                             if "comparison" in r and "similarity_score" in r["comparison"]]
                    if scores:
                        avg_score = sum(scores) / len(scores)
                        f.write(f"\nOVERALL ASSESSMENT\n")
                        f.write("-" * 30 + "\n")
                        f.write(f"Average Similarity Score: {avg_score:.1f}%\n")
                        
                        if avg_score >= 85:
                            f.write("✅ INSPECTION RESULT: PASSED - Excellent quality\n")
                        elif avg_score >= 70:
                            f.write("⚠️ INSPECTION RESULT: PASSED - Good quality\n")
                        elif avg_score >= 50:
                            f.write("⚠️ INSPECTION RESULT: REVIEW NEEDED - Acceptable quality\n")
                        else:
                            f.write("❌ INSPECTION RESULT: FAILED - Poor quality, investigation required\n")
            
            print(f"📄 Analysis report generated: {report_path}")
            
        except Exception as e:
            print(f"❌ Error generating report: {e}")

    def process_all_inspections(self):
        """Main processing loop - scan all new inspection folders"""
        
        last_date = self.read_last_date()
        date_folders = self.list_date_folders()
        
        if not date_folders:
            print("No inspection date folders found")
            return
        
        newest_processed = last_date
        total_processed = 0
        
        for date_folder in date_folders:
            if not self.should_process(date_folder.name, last_date):
                continue
            
            print(f"\n📅 Processing date folder: {date_folder.name}")
            
            # Find all inspection folders in this date folder
            inspection_folders = [p for p in date_folder.iterdir() if p.is_dir()]
            
            for inspection_folder in inspection_folders:
                print(f"\n🔍 Processing inspection: {inspection_folder.name}")
                
                results = self.process_inspection_folder(inspection_folder)
                
                if results:
                    self.generate_analysis_report(inspection_folder, results)
                    total_processed += len([r for r in results if "error" not in r])
                
            # Update newest processed date
            if newest_processed is None or date_folder.name > newest_processed:
                newest_processed = date_folder.name
        
        # Update tracking file
        if newest_processed and newest_processed != last_date:
            self.write_last_date(newest_processed)
            print(f"\n✅ Updated tracking to: {newest_processed}")
        
        print(f"\n🎉 Processing complete! Total PLY files processed: {total_processed}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Enhanced scan processor with PLY cleaning and reference comparison")
    parser.add_argument("--root", type=str, help="FTP root path containing 'inspections' and 'piece_reference' folders (optional, uses LOCAL_FTP_ROOT from .env if not provided)")
    args = parser.parse_args(argv)

    # Use provided root or fallback to environment variable
    if args.root:
        ftp_root = Path(args.root)
    else:
        ftp_root_env = os.getenv('LOCAL_FTP_ROOT')
        if not ftp_root_env:
            print("❌ No FTP root specified. Provide --root argument or set LOCAL_FTP_ROOT in .env file.")
            return 1
        ftp_root = Path(ftp_root_env)

    if not ftp_root.exists():
        print(f"❌ FTP root path not found: {ftp_root}")
        print(f"💡 Make sure the FTP server is running and LOCAL_FTP_ROOT in .env points to the correct directory")
        return 1

    print(f"🚀 Starting Enhanced Scan Processor")
    print(f"📂 FTP Root: {ftp_root}")
    print(f"📂 Piece References: {ftp_root / 'piece_reference'}")
    print(f"🔧 Using LOCAL_FTP_ROOT from .env: {os.getenv('LOCAL_FTP_ROOT', 'Not set')}")
    
    processor = EnhancedScanProcessor(ftp_root)
    processor.process_all_inspections()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())